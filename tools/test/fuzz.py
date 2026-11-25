#!/usr/bin/env python3
"""Differential fuzzer for the MAPF SAT solver encodings.

Generates small random grid instances (movingai format), solves each one for every
requested (objective, motion) with a reference encoding and one or more candidate
encodings (each run in its own fresh working directory, with -p and -l 1 -f), then:
  * validates every plan with validate_plan (start/goal, moves, vertex/swap/pebble),
  * checks the cost computed from the plan equals the cost in the log line,
  * checks the log's LBs equal LBs computed here with BFS,
  * checks every candidate's optimal cost equals the reference's.
Failing instances are kept for reproduction. Exits non-zero on any failure.

Encodings are templates: '{obj}' and '{motion}' are filled per case, e.g.
    --candidates '{obj}_{motion}_pass_eager_single,{obj}_{motion}_at_lazy_single'
A candidate spec may carry extra solver args after a colon:
    --candidate '{obj}_{motion}_shift_eager_dupli:-d 0'
"""

import argparse
import json
import multiprocessing
import os
import random
import shlex
import shutil
import sys
import tempfile
import time
from collections import Counter, deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import solver_runner as sr  # noqa: E402
import validate_plan as vp  # noqa: E402

DEFAULT_REF = "{obj}_{motion}_at_eager_single"
DEFAULT_CANDS = ("{obj}_{motion}_pass_eager_single,{obj}_{motion}_shift_eager_single,"
                 "{obj}_{motion}_at_lazy_single")


# --------------------------------------------------------------------------
# instance generation
# --------------------------------------------------------------------------

def largest_component(free, h, w):
    seen, best = set(), []
    for cell in sorted(free):
        if cell in seen:
            continue
        comp, q = [cell], deque([cell])
        seen.add(cell)
        while q:
            r, c = q.popleft()
            for nb in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
                if nb in free and nb not in seen:
                    seen.add(nb)
                    comp.append(nb)
                    q.append(nb)
        if len(comp) > len(best):
            best = comp
    return set(best)


def gen_instance(rng, a):
    while True:
        h = rng.randint(a.min_size, a.max_size)
        w = rng.randint(a.min_size, a.max_size)
        cells = [(r, c) for r in range(h) for c in range(w)]
        k = int(round(rng.uniform(0.0, a.max_obstacles) * h * w))
        free = set(cells) - set(rng.sample(cells, k))
        free = largest_component(free, h, w)
        if len(free) >= max(2, a.min_agents):
            break
    free_list = sorted(free)
    n = rng.randint(a.min_agents, min(a.max_agents, len(free_list)))
    starts = rng.sample(free_list, n)
    goals, used = [], set()
    for s in starts:
        if rng.random() < a.stay_prob and s not in used:
            g = s
        else:
            g = rng.choice([c for c in free_list if c not in used])
        used.add(g)
        goals.append(g)
    rows = ["".join("." if (r, c) in free else "@" for c in range(w)) for r in range(h)]
    return rows, list(zip(starts, goals))


def write_instance(case_dir, name, rows, agents):
    os.makedirs(case_dir, exist_ok=True)
    h, w = len(rows), len(rows[0])
    map_name = name + ".map"
    with open(os.path.join(case_dir, map_name), "w") as f:
        f.write("type octile\nheight %d\nwidth %d\nmap\n" % (h, w))
        f.write("\n".join(rows) + "\n")
    grid = vp.Grid(h, w, rows)
    scen = os.path.join(case_dir, name + ".scen")
    with open(scen, "w") as f:
        f.write("version 1\n")
        for (s, g) in agents:
            d = grid.bfs(grid.vid[s])[grid.vid[g]]
            # bucket, map, width, height, start col, start row, goal col, goal row, length
            f.write("0\t%s\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n" % (map_name, w, h, s[1], s[0], g[1], g[0], d))
    return scen


# --------------------------------------------------------------------------
# running and checking
# --------------------------------------------------------------------------

def parse_spec(spec):
    """'template[:extra args]' -> (template, [args])."""
    tmpl, _, extra = spec.partition(":")
    return tmpl.strip(), shlex.split(extra)


def check_run(run, enc, obj, motion, grid, agents, lbs):
    """Return (list of (kind, message), cost or None, plan result)."""
    probs = []
    st = run["status"]
    if st == "timeout":
        return [("timeout", "solver timed out")], None, None
    if st != "sat":
        msg = "status %s (rc=%s)" % (st, run["returncode"])
        if run["stderr"].strip():
            msg += "; stderr: " + run["stderr"].strip().splitlines()[-1][:200]
        return [("error", msg)], None, None
    row = run["row"]
    if row.get("encoding") != enc or row.get("agents") != len(agents):
        probs.append(("log", "log row encoding/agents %r/%r" % (row.get("encoding"), row.get("agents"))))
    if (row.get("mks_lb"), row.get("soc_lb")) != lbs:
        probs.append(("lb", "log LBs mks=%s soc=%s, harness BFS gives mks=%d soc=%d"
                      % (row.get("mks_lb"), row.get("soc_lb"), lbs[0], lbs[1])))
    if obj == "mks" and row.get("soc") != 0:
        probs.append(("log", "solution_soc should be 0 for mks objective, got %r" % row.get("soc")))
    cost = sr.objective_value(row, obj)
    parsed = vp.parse_plan_output(run["stdout"])
    res = vp.validate_plan(grid, agents, motion, parsed.paths, max_errors=5,
                           solver_conflicts=parsed.solver_conflicts)
    if len(parsed.blocks) > 1:
        probs.append(("plan", "%d plans printed" % len(parsed.blocks)))
    if not res.ok:
        probs.append(("invalid-plan", "; ".join(res.errors[:5])
                      + (" (+%d more)" % (res.n_errors - 5) if res.n_errors > 5 else "")))
    else:
        plan_cost = res.mks if obj == "mks" else res.soc
        if plan_cost != cost:
            probs.append(("plan-cost", "plan %s=%d but log says %s" % (obj, plan_cost, cost)))
        if obj == "mks" and res.timesteps != cost:
            probs.append(("plan-cost", "plan has %d timesteps but log mks=%s" % (res.timesteps, cost)))
    return probs, cost, res


def run_task(task):
    """Worker entry point: never raises, harness exceptions become failures."""
    try:
        return _run_task(task)
    except Exception:
        import traceback
        return {"case": task["case"], "obj": task["obj"], "motion": task["motion"],
                "case_dir": task["case_dir"],
                "combo_dir": os.path.join(task["case_dir"], "%s_%s" % (task["obj"], task["motion"])),
                "n_agents": len(task["agents"]), "lbs": None, "runs": [], "skipped": None,
                "ref_cost": None, "failures": [{"enc": "-", "role": "harness", "kind": "harness-error",
                                                "msg": traceback.format_exc(limit=3), "repro": "-"}]}


def _run_task(task):
    """One (instance, objective, motion): reference first, then candidates."""
    t = task
    grid = vp.Grid(len(t["rows"]), len(t["rows"][0]), t["rows"])
    agents = t["agents"]
    lbs = vp.lower_bounds(grid, agents)[:2]
    combo = "%s_%s" % (t["obj"], t["motion"])
    combo_dir = os.path.join(t["case_dir"], combo)
    out = {"case": t["case"], "obj": t["obj"], "motion": t["motion"], "case_dir": t["case_dir"],
           "combo_dir": combo_dir, "n_agents": len(agents), "lbs": lbs,
           "failures": [], "runs": [], "skipped": None, "ref_cost": None}

    def do(spec, role):
        tmpl, extra = spec
        label = tmpl + (":" + shlex.join(extra) if extra else "")
        enc = tmpl.format(obj=t["obj"], motion=t["motion"])
        reason = sr.unsupported_reason(enc)
        if reason:
            out["runs"].append({"enc": enc, "label": label, "role": role, "status": "unsupported",
                                "elapsed": 0.0})
            return enc, None, None, None
        base = jobdir = os.path.join(combo_dir, ("ref-" if role == "ref" else "cand-") + enc)
        k = 2
        while os.path.exists(jobdir):  # same encoding listed again with other extra args
            jobdir = "%s.%d" % (base, k)
            k += 1
        timeout = t["timeout"] if role == "ref" else t["cand_timeout"]
        run = sr.run_solver(t["binary"], t["case_dir"], t["scen"], enc, len(agents),
                            timeout, jobdir, extra_args=extra, monosat=t["monosat"])
        probs, cost, _ = check_run(run, enc, t["obj"], t["motion"], grid, agents, lbs)
        out["runs"].append({"enc": enc, "label": label, "role": role, "status": run["status"],
                            "elapsed": run["elapsed"], "cost": cost})
        return enc, run, probs, cost

    enc, run, probs, cost = do(t["ref"], "ref")
    ref_enc = enc
    if run is None:
        out["skipped"] = "reference encoding %s unsupported" % enc
        return out
    if probs == [("timeout", "solver timed out")]:
        out["skipped"] = "reference timed out (unsolvable, or just slow)"
        return out
    for kind, msg in probs:
        out["failures"].append({"enc": enc, "role": "ref", "kind": "ref-" + kind, "msg": msg,
                                "repro": run["repro"]})
    if cost is None:
        return out
    out["ref_cost"] = cost
    for spec in t["cands"]:
        enc, run, probs, ccost = do(spec, "cand")
        if run is None:
            continue
        for kind, msg in probs:
            out["failures"].append({"enc": enc, "role": "cand", "kind": kind, "msg": msg,
                                    "repro": run["repro"]})
        if ccost is not None and ccost != cost:
            out["failures"].append({"enc": enc, "role": "cand", "kind": "cost-mismatch",
                                    "msg": "%s=%d, reference %s gives %d"
                                    % (t["obj"], ccost, ref_enc, cost),
                                    "repro": run["repro"]})
    return out


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--binary", default=sr.DEFAULT_BINARY, help="solver binary (default: %(default)s)")
    ap.add_argument("--monosat", default=sr.DEFAULT_MONOSAT, help="monosat binary linked as ./libs/monosat")
    ap.add_argument("--cases", type=int, default=50, help="number of random instances")
    ap.add_argument("--case-ids", help="only run these case indices (comma list), e.g. to reproduce")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--jobs", type=int, default=4, help="parallel workers (max 8 recommended)")
    ap.add_argument("--timeout", type=int, default=10, help="solver -t for the reference, seconds")
    ap.add_argument("--cand-timeout", type=int, help="solver -t for candidates (default: 2x --timeout)")
    ap.add_argument("--objectives", default="mks,soc")
    ap.add_argument("--motions", default="parallel,pebble")
    ap.add_argument("--reference", default=DEFAULT_REF, help="reference template[:extra args]")
    ap.add_argument("--candidates", default=DEFAULT_CANDS, help="comma-separated candidate templates")
    ap.add_argument("--candidate", action="append", default=[],
                    help="additional candidate 'template[:extra args]' (repeatable)")
    ap.add_argument("--workdir", help="work directory (default: new temp dir)")
    ap.add_argument("--keep", action="store_true", help="keep all instances and run dirs")
    ap.add_argument("--min-size", type=int, default=4)
    ap.add_argument("--max-size", type=int, default=8)
    ap.add_argument("--max-obstacles", type=float, default=0.3)
    ap.add_argument("--min-agents", type=int, default=2)
    ap.add_argument("--max-agents", type=int, default=10)
    ap.add_argument("--stay-prob", type=float, default=0.08, help="probability an agent's goal is its start")
    ap.add_argument("-v", "--verbose", action="store_true", help="print a line for every task")
    a = ap.parse_args(argv)

    binary = os.path.abspath(a.binary)
    if not os.access(binary, os.X_OK):
        ap.error("binary %s is not executable" % binary)
    objs = [o for o in a.objectives.split(",") if o]
    motions = [m for m in a.motions.split(",") if m]
    ref = parse_spec(a.reference)
    cands = [parse_spec(s) for s in a.candidates.split(",") if s.strip()] + [parse_spec(s) for s in a.candidate]
    for o in objs:
        for m in motions:
            for tmpl, _ in [ref] + cands:
                sr.parse_encoding(tmpl.format(obj=o, motion=m))  # fail early on typos

    if a.workdir:
        workdir = os.path.abspath(a.workdir)
        if os.path.exists(workdir) and os.listdir(workdir):
            ap.error("work dir %s exists and is not empty" % workdir)
        os.makedirs(workdir, exist_ok=True)
    else:
        workdir = tempfile.mkdtemp(prefix="mapf_fuzz_")

    ids = range(a.cases) if not a.case_ids else [int(x) for x in a.case_ids.split(",")]
    tasks = []
    for i in ids:
        rng = random.Random("mapf-fuzz:%d:%d" % (a.seed, i))
        rows, agents = gen_instance(rng, a)
        name = "s%d_c%04d" % (a.seed, i)
        case_dir = os.path.join(workdir, name)
        scen = write_instance(case_dir, name, rows, agents)
        for o in objs:
            for m in motions:
                tasks.append({"case": name, "case_dir": case_dir, "scen": scen, "rows": rows,
                              "agents": agents, "obj": o, "motion": m, "ref": ref, "cands": cands,
                              "binary": binary, "monosat": os.path.abspath(a.monosat),
                              "timeout": a.timeout, "cand_timeout": a.cand_timeout or 2 * a.timeout})

    print("fuzz: %d instances x %d combos = %d tasks, %d candidate(s), jobs=%d, workdir=%s"
          % (len(ids), len(objs) * len(motions), len(tasks), len(cands), a.jobs, workdir))
    print("reference: %s; candidates: %s" % (a.reference, ", ".join(
        t + (":" + shlex.join(x) if x else "") for t, x in cands)))
    sys.stdout.flush()

    t0 = time.monotonic()
    results = []
    with multiprocessing.Pool(a.jobs) as pool:
        for k, r in enumerate(pool.imap_unordered(run_task, tasks), 1):
            results.append(r)
            tag = "%s %s/%s a=%d" % (r["case"], r["obj"], r["motion"], r["n_agents"])
            if r["failures"]:
                print("[%d/%d] FAIL %s: %s" % (k, len(tasks), tag,
                      "; ".join("%s %s" % (f["enc"], f["kind"]) for f in r["failures"])))
            elif r["skipped"] and a.verbose:
                print("[%d/%d] SKIP %s: %s" % (k, len(tasks), tag, r["skipped"]))
            elif a.verbose:
                print("[%d/%d] ok   %s cost=%s" % (k, len(tasks), tag, r["ref_cost"]))
            elif k % 50 == 0:
                print("[%d/%d] ... %.0fs" % (k, len(tasks), time.monotonic() - t0))
            sys.stdout.flush()
    wall = time.monotonic() - t0

    # ---- summary
    results.sort(key=lambda r: (r["case"], r["obj"], r["motion"]))
    failed = [r for r in results if r["failures"]]
    skipped = [r for r in results if r["skipped"] and not r["failures"]]
    passed = [r for r in results if not r["failures"] and not r["skipped"]]
    kinds = Counter(f["kind"] for r in failed for f in r["failures"])
    stats = {}
    for r in results:
        for run in r["runs"]:
            key = ("ref " if run["role"] == "ref" else "") + run["label"]
            s = stats.setdefault(key, Counter())
            s["runs"] += 1
            s[run["status"]] += 1
            s["time"] += run["elapsed"]

    lines = []
    lines.append("=" * 72)
    lines.append("tasks: %d  passed: %d  skipped: %d  FAILED: %d  (wall %.1fs)"
                 % (len(results), len(passed), len(skipped), len(failed), wall))
    combos = Counter((r["obj"], r["motion"]) for r in passed)
    lines.append("passed per combo: " + ", ".join("%s/%s=%d" % (o, m, c) for (o, m), c in sorted(combos.items())))
    if skipped:
        lines.append("skipped reasons: " + ", ".join("%s=%d" % kv for kv in Counter(r["skipped"] for r in skipped).items()))
        solved = {(r["case"], r["obj"], r["motion"]): r["ref_cost"] for r in passed}
        n_solvable = 0
        for r in skipped:
            other = [(o, m, c) for (cs, o, m), c in solved.items() if cs == r["case"]
                     and (m == r["motion"] or (m == "pebble" and r["motion"] == "parallel"))]
            note = ""
            if other:  # solvable under another objective / a stricter motion => just slow
                n_solvable += 1
                note = "  (instance IS solvable: %s)" % ", ".join("%s/%s=%s" % x for x in sorted(other))
            lines.append("  skip %s %s/%s agents=%d LB(mks,soc)=%s%s"
                         % (r["case"], r["obj"], r["motion"], r["n_agents"], r["lbs"], note))
        lines.append("  -> %d of %d skipped tasks are known-solvable (reference just too slow)"
                     % (n_solvable, len(skipped)))
    lines.append("per encoding template: runs / sat / timeout / other / avg s")
    for key in sorted(stats):
        s = stats[key]
        other = s["runs"] - s["sat"] - s["timeout"]
        lines.append("  %-44s %5d %5d %5d %5d %7.3f" % (key, s["runs"], s["sat"], s["timeout"], other,
                                                       s["time"] / max(1, s["runs"])))
    if kinds:
        lines.append("failure kinds: " + ", ".join("%s=%d" % kv for kv in kinds.most_common()))
    for r in failed:
        lines.append("-" * 72)
        lines.append("FAIL %s %s/%s agents=%d LB(mks,soc)=%s ref_cost=%s"
                     % (r["case"], r["obj"], r["motion"], r["n_agents"], r["lbs"], r["ref_cost"]))
        lines.append("  instance: %s" % r["case_dir"])
        for f in r["failures"]:
            lines.append("  [%s] %s: %s" % (f["kind"], f["enc"], f["msg"]))
            lines.append("    repro: %s" % f["repro"])
    report = "\n".join(lines)
    print(report)
    with open(os.path.join(workdir, "report.txt"), "w") as f:
        f.write(report + "\n")
    with open(os.path.join(workdir, "failures.json"), "w") as f:
        json.dump([{k: r[k] for k in ("case", "obj", "motion", "case_dir", "lbs", "ref_cost", "failures")}
                   for r in failed], f, indent=1)

    # ---- cleanup: keep failing instances (and everything with --keep)
    if not a.keep:
        bad_cases = {r["case"] for r in failed}
        for r in results:
            if r["case"] not in bad_cases and os.path.isdir(r["case_dir"]):
                shutil.rmtree(r["case_dir"], ignore_errors=True)
        for r in failed:  # in failing cases, drop the passing combos' run dirs
            for other in results:
                if other["case"] == r["case"] and not other["failures"]:
                    shutil.rmtree(other["combo_dir"], ignore_errors=True)
    if failed or a.keep:
        print("instances kept in %s (report.txt, failures.json)" % workdir)
    else:
        print("all passed; report in %s/report.txt" % workdir)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
