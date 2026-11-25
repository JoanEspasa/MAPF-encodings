"""Shared helpers for running the MAPF solver binary and parsing its log.

Used by fuzz.py and oracle.py. Standard library only.
"""

import os
import shlex
import signal
import subprocess
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
DEFAULT_BINARY = os.path.join(REPO, "release", "MAPF")
DEFAULT_MONOSAT = os.path.join(REPO, "libs", "monosat")

# Columns of a `-l 1` log line (1-based in the docs, 0-based here).
LOG_FIELDS = [
    "map", "scen", "encoding", "agents", "mks_lb", "soc_lb", "mks", "soc",
    "build_ms", "solve_ms", "solver_calls", "nr_vars", "nr_clauses",
    "move", "dupli", "conflict", "soc_clauses", "unit", "status",
]
INT_FIELDS = set(LOG_FIELDS[3:18])

OBJECTIVES = ("mks", "soc")
MOTIONS = ("parallel", "pebble")
VARIABLES = ("at", "pass", "shift", "monosat-pass", "monosat-shift", "up")


def parse_encoding(enc):
    """Split a current-style encoding name into its 5 parts (raises ValueError)."""
    parts = enc.split("_")
    if len(parts) != 5:
        raise ValueError("encoding %r must have 5 '_'-separated parts" % enc)
    obj, motion, var, lazy, dupli = parts
    if (obj not in OBJECTIVES or motion not in MOTIONS or var not in VARIABLES
            or lazy not in ("eager", "lazy") or dupli not in ("single", "dupli")):
        raise ValueError("unknown encoding %r" % enc)
    return {"obj": obj, "motion": motion, "var": var, "lazy": lazy, "dupli": dupli}


def is_monosat(enc):
    return "monosat" in enc


def unsupported_reason(enc):
    """Return a reason string if the encoding is a known-unsupported combo, else None."""
    p = parse_encoding(enc)
    if p["var"].startswith("monosat") and (p["motion"] != "parallel" or p["lazy"] != "eager"):
        return "monosat encodings only support parallel+eager"
    if p["var"] == "up" and p["lazy"] != "eager":
        return "up encodings are eager only"
    return None


def parse_log_line(line):
    """Parse one tab-separated log line into a dict (None for blank lines)."""
    line = line.rstrip("\r\n")
    if not line.strip():
        return None
    fields = line.split("\t")
    while fields and fields[-1] == "":
        fields.pop()
    row = {"raw": line, "nfields": len(fields)}
    for i, name in enumerate(LOG_FIELDS[:-1]):
        if i < len(fields):
            val = fields[i]
            if name in INT_FIELDS:
                try:
                    val = int(val)
                except ValueError:
                    pass
            row[name] = val
    # status is the last non-empty field (robust to older/shorter formats)
    row["status"] = fields[-1] if fields else ""
    return row


def parse_log_file(path):
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path, errors="replace") as f:
        for line in f:
            row = parse_log_line(line)
            if row is not None:
                rows.append(row)
    return rows


def objective_value(row, obj):
    """The optimal cost reported in a log row for objective obj (col 7 / col 8)."""
    return row.get("mks") if obj == "mks" else row.get("soc")


def run_solver(binary, map_dir, scen, encoding, agents, timeout, jobdir,
               extra_args=(), monosat=DEFAULT_MONOSAT, print_plan=True,
               hard_slack=None):
    """Run one solver invocation in a fresh working directory `jobdir`.

    Returns a dict with keys: status (sat|unsat|timeout|killed|crash|nolog|multilog),
    row (parsed log line or None), stdout, stderr, returncode, elapsed, cmd, repro,
    jobdir.
    """
    if os.path.exists(jobdir) and os.listdir(jobdir):
        raise RuntimeError("job dir %s is not empty" % jobdir)
    os.makedirs(jobdir, exist_ok=True)
    binary = os.path.abspath(binary)
    map_dir = os.path.abspath(map_dir)
    scen = os.path.abspath(scen)

    cmd = [binary, "-m", map_dir, "-s", scen, "-e", encoding, "-a", str(agents),
           "-t", str(timeout), "-l", "1", "-f", "result.res"]
    cmd.append("-p" if print_plan else "-q")
    if is_monosat(encoding):
        # monosat encodings call ./libs/monosat relative to CWD and need -c
        os.makedirs(os.path.join(jobdir, "libs"), exist_ok=True)
        os.symlink(os.path.abspath(monosat), os.path.join(jobdir, "libs", "monosat"))
        if "-c" not in extra_args:
            cmd += ["-c", "formula.cnf"]
    cmd += list(extra_args)

    repro = "cd %s && %s" % (shlex.quote(jobdir), shlex.join(cmd))
    with open(os.path.join(jobdir, "cmd.sh"), "w") as f:
        f.write("#!/bin/sh\n# reproduce this run (expects ./libs/monosat for monosat encodings)\n")
        f.write(repro + "\n")

    if hard_slack is None:
        hard_slack = 30 + timeout // 2
    t0 = time.monotonic()
    killed = False
    proc = subprocess.Popen(cmd, cwd=jobdir, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, start_new_session=True)
    try:
        out, err = proc.communicate(timeout=timeout + hard_slack)
    except subprocess.TimeoutExpired:
        killed = True
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        out, err = proc.communicate()
    elapsed = time.monotonic() - t0
    stdout = out.decode(errors="replace")
    stderr = err.decode(errors="replace")
    with open(os.path.join(jobdir, "stdout.txt"), "w") as f:
        f.write(stdout)
    if stderr:
        with open(os.path.join(jobdir, "stderr.txt"), "w") as f:
            f.write(stderr)

    rows = parse_log_file(os.path.join(jobdir, "result.res"))
    row = rows[-1] if rows else None
    if killed:
        status = "killed"
    elif proc.returncode != 0:
        status = "crash"
    elif len(rows) > 1:
        status = "multilog"
    elif row is not None:
        status = row["status"] if row["status"] in ("sat", "unsat") else "badlog"
    elif "No solution found in the given timeout" in stdout:
        status = "timeout"
    else:
        status = "nolog"

    # CNF files of monosat runs can be big; they are reproducible from cmd.sh
    cnf = os.path.join(jobdir, "formula.cnf")
    if os.path.exists(cnf):
        os.remove(cnf)

    return {"status": status, "row": row, "rows": rows, "stdout": stdout,
            "stderr": stderr, "returncode": proc.returncode, "elapsed": elapsed,
            "cmd": cmd, "repro": repro, "jobdir": jobdir, "encoding": encoding}
