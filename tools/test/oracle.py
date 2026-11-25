#!/usr/bin/env python3
"""Optimal-cost oracle built from the published AAMAS2026 experiment results.

Loads every *.res under experiments/AAMAS2026 (tab separated; first 8 columns
map, scen, encoding, agents, mksLB, socLB, mks, soc; old encoding names such as
'monosat-pass_parallel_soc_all'). Keeps rows whose last non-empty field is 'sat'
with solver_calls (col 11) > 0 (results_kissat/results_50agents.res is a
build-only experiment: 0 solver calls, 'solution' = lower bound, still marked
'sat') and builds (scenario basename, agents, objective, motion) -> optimal value
(col 7 for mks, col 8 for soc). All published rows use parallel motion, so rows
of ours with pebble motion get NO-ORACLE. Lower bounds (cols 5/6) are checked
too: a LB difference means different instance files (the repo's instances/ dir
has the same names but different contents than the paper's).

Usage:
  oracle.py --check                          self-consistency + table size
  oracle.py RESULTS.res [...]                compare our log files (-l 1 -f)
  oracle.py --run -i random_20_0:10,15,20 -i empty_40_0:20 --binary B [--objectives mks,soc]
                                             run the solver on paper instances
                                             (each job in its own dir), validate
                                             the plans and compare.
"""

import argparse
import glob
import os
import sys
from collections import Counter, defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import solver_runner as sr  # noqa: E402

DEFAULT_ROOT = os.path.join(sr.REPO, "experiments", "AAMAS2026")
PAPER_INSTANCES = os.path.join(DEFAULT_ROOT, "MAPF_Encodings", "instances")


def enc_objective(enc):
    toks = enc.split("_")
    objs = [t for t in toks if t in ("mks", "soc")]
    return objs[0] if len(objs) == 1 else None


def enc_motion(enc):
    toks = enc.split("_")
    return "pebble" if "pebble" in toks else "parallel"


def iter_rows(path):
    """Yield (lineno, row-dict or None, raw fields) for a result file."""
    with open(path, errors="replace") as f:
        for lineno, line in enumerate(f, 1):
            line = line.rstrip("\r\n")
            if not line.strip():
                continue
            fields = line.split("\t")
            while fields and fields[-1].strip() == "":
                fields.pop()
            row = None
            if len(fields) >= 9:
                try:
                    row = {"map": fields[0], "scen": os.path.basename(fields[1]),
                           "encoding": fields[2], "agents": int(fields[3]),
                           "mks_lb": int(fields[4]), "soc_lb": int(fields[5]),
                           "mks": int(fields[6]), "soc": int(fields[7]),
                           "calls": int(fields[10]) if len(fields) > 11 else None,
                           "status": fields[-1].strip()}
                except ValueError:
                    row = None
            yield lineno, row, fields


class Oracle:
    def __init__(self, root=DEFAULT_ROOT):
        self.root = root
        self.files = sorted(glob.glob(os.path.join(root, "**", "*.res"), recursive=True))
        self.values = defaultdict(lambda: defaultdict(list))  # key -> value -> [source]
        self.lbs = defaultdict(lambda: defaultdict(list))     # (scen, agents) -> (mksLB, socLB) -> [source]
        self.stats = Counter()
        for path in self.files:
            rel = os.path.relpath(path, root)
            for lineno, row, fields in iter_rows(path):
                self.stats["lines"] += 1
                if row is None:
                    self.stats["malformed"] += 1
                    continue
                if row["status"] != "sat":
                    self.stats["not-sat"] += 1
                    continue
                if row["calls"] == 0:
                    # e.g. results_kissat/results_50agents.res: build-only runs marked 'sat'
                    # with 0 solver calls whose 'solution' is just the lower bound
                    self.stats["zero-calls"] += 1
                    continue
                obj = enc_objective(row["encoding"])
                if obj is None:
                    self.stats["no-objective"] += 1
                    continue
                self.stats["sat"] += 1
                src = "%s:%d (%s)" % (rel, lineno, row["encoding"])
                key = (row["scen"], row["agents"], obj, enc_motion(row["encoding"]))
                self.values[key][row["mks"] if obj == "mks" else row["soc"]].append(src)
                self.lbs[(row["scen"], row["agents"])][(row["mks_lb"], row["soc_lb"])].append(src)

    def __len__(self):
        return len(self.values)

    def disagreements(self):
        return {k: v for k, v in self.values.items() if len(v) > 1}

    def lb_disagreements(self):
        return {k: v for k, v in self.lbs.items() if len(v) > 1}

    def lookup(self, scen, agents, obj, motion="parallel"):
        """Return the set of published optimal values (empty if unknown)."""
        return set(self.values.get((os.path.basename(scen), agents, obj, motion), {}))

    def lookup_lbs(self, scen, agents):
        return set(self.lbs.get((os.path.basename(scen), agents), {}))

    def report_consistency(self, out=sys.stdout, max_show=20):
        s = self.stats
        print("oracle: %d files under %s" % (len(self.files), self.root), file=out)
        print("  lines=%d used=%d not-sat=%d zero-solver-calls(ignored)=%d malformed=%d no-objective=%d"
              % (s["lines"], s["sat"], s["not-sat"], s["zero-calls"], s["malformed"], s["no-objective"]),
              file=out)
        per_obj = Counter(k[2] for k in self.values)
        print("  table size: %d keys (mks=%d, soc=%d), %d scenarios, %d (scen,agents) LB entries"
              % (len(self), per_obj["mks"], per_obj["soc"], len({k[0] for k in self.values}),
                 len(self.lbs)), file=out)
        dis, lbdis = self.disagreements(), self.lb_disagreements()
        print("  value disagreements: %d; LB disagreements: %d" % (len(dis), len(lbdis)), file=out)
        for k in sorted(dis)[:max_show]:
            print("    VALUE %s a=%d %s/%s: %s" % (k[0], k[1], k[2], k[3], "; ".join(
                "%d <- %s%s" % (v, srcs[0], " +%d" % (len(srcs) - 1) if len(srcs) > 1 else "")
                for v, srcs in sorted(dis[k].items()))), file=out)
        for k in sorted(lbdis)[:max_show]:
            print("    LB %s a=%d: %s" % (k[0], k[1], "; ".join(
                "%s <- %s" % (v, srcs[0]) for v, srcs in sorted(lbdis[k].items()))), file=out)
        return len(dis) + len(lbdis)


def compare_files(oracle, files, out=sys.stdout):
    counts = Counter()
    for path in files:
        for lineno, row, fields in iter_rows(path):
            where = "%s:%d" % (path, lineno)
            if row is None:
                counts["MALFORMED"] += 1
                print("MALFORMED  %s" % where, file=out)
                continue
            obj = enc_objective(row["encoding"])
            motion = enc_motion(row["encoding"])
            ours = row["mks"] if obj == "mks" else row["soc"]
            tag = "%-22s a=%-3d %s/%-8s %-34s" % (row["scen"], row["agents"], obj, motion, row["encoding"])
            if row["status"] != "sat":
                counts["SKIPPED"] += 1
                print("SKIPPED    %s status=%s  %s" % (tag, row["status"], where), file=out)
                continue
            vals = oracle.lookup(row["scen"], row["agents"], obj, motion)
            lbs = oracle.lookup_lbs(row["scen"], row["agents"])
            lb_bad = lbs and (row["mks_lb"], row["soc_lb"]) not in lbs
            if not vals:
                status, info = "NO-ORACLE", "ours=%d" % ours
                if lb_bad:
                    status, info = "MISMATCH", info + " LB=%s, oracle LB=%s (different instance files?)" % (
                        (row["mks_lb"], row["soc_lb"]), sorted(lbs))
            elif ours in vals and not lb_bad:
                status, info = "OK", "ours=%d oracle=%d" % (ours, ours)
            else:
                status = "MISMATCH"
                info = "ours=%d oracle=%s" % (ours, "/".join(map(str, sorted(vals))))
                if lb_bad:
                    info += " LB=%s, oracle LB=%s (different instance files?)" % (
                        (row["mks_lb"], row["soc_lb"]), sorted(lbs))
            if len(vals) > 1:
                info += " [oracle ambiguous]"
            counts[status] += 1
            print("%-10s %s %s  %s" % (status, tag, info, where), file=out)
    total = sum(counts.values())
    print("summary: %d rows: %s" % (total, ", ".join(
        "%s=%d" % (k, counts[k]) for k in ("OK", "MISMATCH", "NO-ORACLE", "SKIPPED", "MALFORMED") if counts[k])),
        file=out)
    return counts


def _run_job(job):
    import validate_plan as vp
    run = sr.run_solver(job["binary"], job["map_dir"], job["scen"], job["enc"], job["agents"],
                        job["timeout"], job["jobdir"], monosat=job["monosat"], print_plan=True)
    msg = ""
    if run["status"] == "sat":
        obj, motion = enc_objective(job["enc"]), enc_motion(job["enc"])
        map_name, agents = vp.load_scen(job["scen"])
        grid = vp.load_map(os.path.join(job["map_dir"], map_name))
        parsed = vp.parse_plan_output(run["stdout"])
        res = vp.validate_plan(grid, agents[:job["agents"]], motion, parsed.paths,
                               solver_conflicts=parsed.solver_conflicts)
        logged = sr.objective_value(run["row"], obj)
        if not res.ok:
            msg = "PLAN INVALID: " + "; ".join(res.errors[:3])
        elif (res.mks if obj == "mks" else res.soc) != logged:
            msg = "PLAN COST %d != logged %d" % (res.mks if obj == "mks" else res.soc, logged)
        else:
            msg = "plan valid (%s=%d)" % (obj, logged)
    return {"job": job, "status": run["status"], "elapsed": run["elapsed"], "rows": run["rows"],
            "msg": msg, "repro": run["repro"]}


def run_paper(args, oracle):
    import multiprocessing
    import tempfile
    binary = os.path.abspath(args.binary)
    workdir = os.path.abspath(args.workdir) if args.workdir else tempfile.mkdtemp(prefix="mapf_oracle_")
    os.makedirs(workdir, exist_ok=True)
    jobs = []
    for spec in args.instance:
        name, _, ags = spec.partition(":")
        name = name[:-5] if name.endswith(".scen") else name
        scen = os.path.join(args.instances, "scenarios", name + ".scen")
        if not os.path.isfile(scen):
            sys.exit("no such paper scenario: %s" % scen)
        for a in [int(x) for x in ags.split(",") if x] or [5]:
            for obj in args.objectives.split(","):
                for motion in args.motions.split(","):
                    for tmpl in args.encodings.split(","):
                        enc = tmpl.format(obj=obj, motion=motion)
                        sr.parse_encoding(enc)
                        jobdir = os.path.join(workdir, "%s_a%d_%s" % (name, a, enc))
                        jobs.append({"binary": binary, "map_dir": os.path.join(args.instances, "maps"),
                                     "scen": scen, "enc": enc, "agents": a, "timeout": args.timeout,
                                     "jobdir": jobdir, "monosat": os.path.abspath(args.monosat)})
    print("running %d paper jobs (jobs=%d, timeout=%ds) in %s" % (len(jobs), args.jobs, args.timeout, workdir))
    sys.stdout.flush()
    with multiprocessing.Pool(args.jobs) as pool:
        results = list(pool.imap(_run_job, jobs))
    out_file = os.path.join(workdir, "results.res")
    bad_plans = 0
    with open(out_file, "w") as f:
        for r in results:
            j = r["job"]
            print("  %-16s a=%-3d %-34s %-8s %6.1fs  %s" % (os.path.basename(j["scen"]), j["agents"], j["enc"],
                                                         r["status"], r["elapsed"], r["msg"]))
            if r["msg"].startswith("PLAN"):
                bad_plans += 1
                print("    repro: %s" % r["repro"])
            for row in r["rows"]:
                f.write(row["raw"] + "\n")
    print("combined log: %s" % out_file)
    counts = compare_files(oracle, [out_file])
    return counts, bad_plans


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="*", help="our result files (-l 1 -f output)")
    ap.add_argument("--root", default=DEFAULT_ROOT, help="directory with published *.res (default: %(default)s)")
    ap.add_argument("--check", action="store_true", help="report oracle self-consistency and size")
    g = ap.add_argument_group("--run: solve paper instances and compare")
    g.add_argument("--run", action="store_true")
    g.add_argument("-i", "--instance", action="append", default=[],
                   help="SCEN_NAME:AGENTS[,AGENTS...], e.g. random_20_0:10,15,20 (repeatable)")
    g.add_argument("--binary", default=sr.DEFAULT_BINARY)
    g.add_argument("--monosat", default=sr.DEFAULT_MONOSAT)
    g.add_argument("--instances", default=PAPER_INSTANCES, help="dir with maps/ and scenarios/ (default: paper's)")
    g.add_argument("--objectives", default="mks,soc")
    g.add_argument("--motions", default="parallel")
    g.add_argument("--encodings", default="{obj}_{motion}_at_eager_single", help="comma-separated templates")
    g.add_argument("--timeout", type=int, default=60)
    g.add_argument("--jobs", type=int, default=4)
    g.add_argument("--workdir")
    args = ap.parse_args(argv)

    oracle = Oracle(args.root)
    n_incons = oracle.report_consistency()
    rc = 0
    if args.check and n_incons:
        rc = 1
    if args.run:
        if not args.instance:
            ap.error("--run needs at least one --instance")
        counts, bad_plans = run_paper(args, oracle)
        if counts["MISMATCH"] or bad_plans:
            rc = 1
    if args.files:
        counts = compare_files(oracle, args.files)
        if counts["MISMATCH"]:
            rc = 1
    if not (args.check or args.run or args.files):
        ap.print_help()
    return rc


if __name__ == "__main__":
    sys.exit(main())
