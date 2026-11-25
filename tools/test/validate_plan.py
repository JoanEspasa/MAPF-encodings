#!/usr/bin/env python3
"""Validate a plan printed by the MAPF solver (`MAPF -p`).

Library use:
    grid = load_map(map_path)
    map_name, agents = load_scen(scen_path)
    parsed = parse_plan_output(stdout_text)
    res = validate_plan(grid, agents[:n], motion, parsed.paths)
    res.ok, res.errors, res.mks, res.soc, res.costs

CLI:
    validate_plan.py -m MAP_OR_MAP_DIR -s SCEN [-a N] [--motion parallel|pebble] [PLAN|-]

Conventions (mirroring src/instance.cpp and src/encodings/solver_common.cpp):
  * map: only '.' is free; free cells are numbered row-major from 0.
  * scen: first line is a header; start = (row=col5, col=col4), goal = (row=col7,
    col=col6), columns 0-based, tab separated.
  * plan: `Agent #i : v0 v1 ... v{T-1}` with internal vertex ids. Under SoC the
    solver normalizes the plan so entries after an agent finished are its goal.
  * cost of an agent = (first timestep from which it stays at its goal) + 1,
    mks = max cost, soc = sum of costs; LBs are max/sum of (shortest path + 1).
  * pebble rule (VerifyPlan/GenerateConflicts): an agent may not move at t+1 into
    a vertex that another agent occupied at t.
"""

import argparse
import os
import re
import sys
from collections import deque

__all__ = ["Grid", "load_map", "load_scen", "parse_plan_output", "validate_plan",
           "lower_bounds", "ValidationResult"]


class Grid:
    def __init__(self, height, width, rows):
        self.height = height
        self.width = width
        self.rows = rows  # list of strings, '.' free
        self.vid = {}
        self.coords = []
        for r in range(height):
            for c in range(width):
                if rows[r][c] == ".":
                    self.vid[(r, c)] = len(self.coords)
                    self.coords.append((r, c))
        self.adj = []
        for (r, c) in self.coords:
            nb = []
            for (dr, dc) in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                u = self.vid.get((r + dr, c + dc))
                if u is not None:
                    nb.append(u)
            self.adj.append(nb)

    @property
    def nr_vertices(self):
        return len(self.coords)

    def bfs(self, src):
        dist = [-1] * len(self.coords)
        dist[src] = 0
        q = deque([src])
        while q:
            v = q.popleft()
            for u in self.adj[v]:
                if dist[u] < 0:
                    dist[u] = dist[v] + 1
                    q.append(u)
        return dist

    def describe(self, v):
        if v is None or not (0 <= v < len(self.coords)):
            return "%s(invalid)" % v
        r, c = self.coords[v]
        return "%d(r%d,c%d)" % (v, r, c)


def load_map(path):
    """Parse a movingai map exactly like _MAPFSAT_Instance::LoadMap."""
    with open(path) as f:
        f.readline()  # type line
        tokens = f.read().split()
    kv = {}
    i = 0
    while i < len(tokens) and tokens[i] != "map":
        if i + 1 < len(tokens):
            kv[tokens[i]] = tokens[i + 1]
        i += 2
    height, width = int(kv["height"]), int(kv["width"])
    chars = "".join(tokens[i + 1:])
    if len(chars) < height * width:
        raise ValueError("map %s: expected %d cells, got %d" % (path, height * width, len(chars)))
    rows = [chars[r * width:(r + 1) * width] for r in range(height)]
    return Grid(height, width, rows)


def load_scen(path):
    """Return (map_name, [((sr, sc), (gr, gc)), ...]) like LoadAgents."""
    agents = []
    map_name = None
    with open(path) as f:
        f.readline()  # version header
        for line in f:
            line = line.rstrip("\r\n")
            if not line.strip():
                continue
            p = line.split("\t")
            if map_name is None:
                map_name = p[1]
            start = (int(p[5]), int(p[4]))
            goal = (int(p[7]), int(p[6]))
            agents.append((start, goal))
    return map_name, agents


def resolve_map(map_arg, scen_path, map_name):
    """map_arg may be a map file, a map directory, or None (guess from scen location)."""
    if map_arg and os.path.isfile(map_arg):
        return map_arg
    cands = []
    if map_arg:
        cands.append(os.path.join(map_arg, map_name))
    else:
        d = os.path.dirname(os.path.abspath(scen_path))
        cands += [os.path.join(d, map_name), os.path.join(d, "..", "maps", map_name)]
    for c in cands:
        if os.path.isfile(c):
            return c
    raise FileNotFoundError("cannot find map %s (tried %s)" % (map_name, ", ".join(cands)))


def lower_bounds(grid, agents):
    """(mks_LB, soc_LB, [SP_i]) with the solver's +1 convention; SP=-1 if unreachable."""
    sps = []
    for (s, g) in agents:
        sv, gv = grid.vid.get(s), grid.vid.get(g)
        if sv is None or gv is None:
            sps.append(-1)
            continue
        sps.append(grid.bfs(sv)[gv])
    mks = max([sp + 1 for sp in sps], default=0)
    soc = sum(sp + 1 for sp in sps)
    return mks, soc, sps


class ParsedOutput:
    def __init__(self):
        self.blocks = []            # list of dicts: header (agents, timesteps) | None, paths
        self.solver_conflicts = []  # lines printed by the solver's own VerifyPlan
        self.encoding = None
        self.errors = []            # parse problems

    @property
    def paths(self):
        """Paths of the last plan block as a list indexed by agent (None if missing)."""
        if not self.blocks:
            return None
        d = self.blocks[-1]["paths"]
        if not d:
            return []
        n = max(d) + 1
        return [d.get(a) for a in range(n)]

    @property
    def header(self):
        return self.blocks[-1]["header"] if self.blocks else None


_HDR = re.compile(r"^Found plan \[agents = (\d+)\] \[timesteps = (\d+)\]")
_AGENT = re.compile(r"^Agent #(\d+) :(.*)$")
_CONFLICT = re.compile(r"(Vertex|Swapping|Pebble) conflict!")
_ENC = re.compile(r"^Encoding used: (\S+)")


def parse_plan_output(text):
    out = ParsedOutput()
    cur = None
    for lineno, line in enumerate(text.splitlines(), 1):
        line = line.rstrip()
        m = _ENC.match(line)
        if m:
            out.encoding = m.group(1)
            continue
        if _CONFLICT.search(line):
            out.solver_conflicts.append(line)
            continue
        m = _HDR.match(line)
        if m:
            cur = {"header": (int(m.group(1)), int(m.group(2))), "paths": {}}
            out.blocks.append(cur)
            continue
        m = _AGENT.match(line)
        if m:
            if cur is None:  # bare agent lines without the header
                cur = {"header": None, "paths": {}}
                out.blocks.append(cur)
            a = int(m.group(1))
            try:
                path = [int(x) for x in m.group(2).split()]
            except ValueError:
                out.errors.append("line %d: non-integer entry in %r" % (lineno, line))
                path = []
            if a in cur["paths"]:
                out.errors.append("line %d: agent #%d listed twice" % (lineno, a))
            cur["paths"][a] = path
            continue
    return out


class ValidationResult:
    def __init__(self):
        self.errors = []
        self.n_errors = 0
        self.costs = []
        self.mks = None
        self.soc = None
        self.timesteps = None
        self.mks_lb = None
        self.soc_lb = None

    @property
    def ok(self):
        return self.n_errors == 0

    def summary(self):
        return ("agents=%d timesteps=%s mks=%s soc=%s (LB mks=%s soc=%s)"
                % (len(self.costs), self.timesteps, self.mks, self.soc, self.mks_lb, self.soc_lb))


def validate_plan(grid, agents, motion, paths, max_errors=50, solver_conflicts=()):
    """Validate `paths` (list over agents of vertex-id lists) for the given agents.

    agents: list of ((sr, sc), (gr, gc)) of exactly the agents that were solved.
    motion: 'parallel' or 'pebble'.
    """
    res = ValidationResult()

    def err(msg):
        res.n_errors += 1
        if len(res.errors) < max_errors:
            res.errors.append(msg)

    if motion not in ("parallel", "pebble"):
        raise ValueError("motion must be parallel or pebble")
    res.mks_lb, res.soc_lb, _ = lower_bounds(grid, agents)
    for line in solver_conflicts:
        err("solver reported: " + line.strip())

    n = len(agents)
    if paths is None:
        err("no plan found in output")
        return res
    if len(paths) != n:
        err("plan has %d agents, expected %d" % (len(paths), n))
    nv = grid.nr_vertices
    P = []
    for a in range(n):
        p = paths[a] if a < len(paths) else None
        if not p:
            err("agent %d: missing or empty path" % a)
            p = None
        else:
            q = []
            for t, v in enumerate(p):
                if 0 <= v < nv:
                    q.append(v)
                else:
                    err("agent %d: invalid vertex id %d at t=%d" % (a, v, t))
                    q.append(None)
            p = q
        P.append(p)
    lengths = {len(p) for p in P if p}
    if len(lengths) > 1:
        err("agents have different plan lengths: %s" % sorted(lengths))
    T = max(lengths, default=0)
    res.timesteps = T

    # start / goal / movement
    for a, ((s, g), p) in enumerate(zip(agents, P)):
        sv, gv = grid.vid.get(s), grid.vid.get(g)
        if sv is None:
            err("agent %d: scenario start %s is not a free cell" % (a, s))
        if gv is None:
            err("agent %d: scenario goal %s is not a free cell" % (a, g))
        if p is None:
            continue
        if p[0] != sv:
            err("agent %d: starts at %s, expected start %s" % (a, grid.describe(p[0]), grid.describe(sv)))
        if p[-1] != gv:
            err("agent %d: ends at %s, expected goal %s" % (a, grid.describe(p[-1]), grid.describe(gv)))
        for t in range(len(p) - 1):
            u, v = p[t], p[t + 1]
            if u is None or v is None or u == v:
                continue
            if v not in grid.adj[u]:
                err("agent %d: illegal move %s -> %s at t=%d->%d"
                    % (a, grid.describe(u), grid.describe(v), t, t + 1))

    # pad shorter paths with their last position (agents wait at the end)
    def pos(a, t):
        p = P[a]
        if p is None:
            return None
        return p[t] if t < len(p) else p[-1]

    live = [a for a in range(n) if P[a] is not None]
    for t in range(T):
        occ = {}
        for a in live:
            v = pos(a, t)
            if v is None:
                continue
            if v in occ:
                err("vertex conflict: agents %d and %d at %s, t=%d" % (occ[v], a, grid.describe(v), t))
            else:
                occ[v] = a
        if t + 1 >= T:
            continue
        moves = {}
        for a in live:
            u, v = pos(a, t), pos(a, t + 1)
            if u is not None and v is not None and u != v:
                moves[(u, v)] = a
        for (u, v), a in moves.items():
            b = moves.get((v, u))
            if b is not None and a < b:
                err("swap conflict: agents %d and %d swap over edge %s-%s at t=%d->%d"
                    % (a, b, grid.describe(u), grid.describe(v), t, t + 1))
            if motion == "pebble":
                b = occ.get(v)
                if b is not None and b != a:
                    err("pebble conflict: agent %d moves into %s at t=%d but agent %d was there at t=%d"
                        % (a, grid.describe(v), t + 1, b, t))

    # costs
    if res.n_errors == 0:
        for a, ((s, g), p) in enumerate(zip(agents, P)):
            gv = grid.vid[g]
            k = len(p)
            while k > 0 and p[k - 1] == gv:
                k -= 1
            res.costs.append(k + 1)  # first index from which it stays at goal, +1
        res.mks = max(res.costs, default=0)
        res.soc = sum(res.costs)
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description="Validate a plan printed by `MAPF -p`.")
    ap.add_argument("-m", "--map", help="map file or map directory (default: next to the scen or ../maps)")
    ap.add_argument("-s", "--scen", required=True, help="scenario file")
    ap.add_argument("-a", "--agents", type=int, help="number of agents (default: as many as in the plan)")
    ap.add_argument("--motion", choices=("parallel", "pebble"),
                    help="motion model (default: inferred from 'Encoding used:' in the output)")
    ap.add_argument("--expect-mks", type=int, help="also require the plan's makespan to equal this")
    ap.add_argument("--expect-soc", type=int, help="also require the plan's sum of costs to equal this")
    ap.add_argument("--max-errors", type=int, default=20, help="number of violations to print")
    ap.add_argument("plan", nargs="?", default="-", help="file with solver stdout (default: stdin)")
    args = ap.parse_args(argv)

    text = sys.stdin.read() if args.plan == "-" else open(args.plan).read()
    parsed = parse_plan_output(text)
    if not parsed.blocks:
        print("ERROR: no plan found (run the solver with -p and without -q)")
        return 2
    if len(parsed.blocks) > 1:
        print("note: %d plans in output, validating the last one" % len(parsed.blocks))
    motion = args.motion
    if motion is None and parsed.encoding:
        motion = parsed.encoding.split("_")[1] if parsed.encoding.count("_") >= 1 else None
    if motion not in ("parallel", "pebble"):
        print("ERROR: cannot infer motion, pass --motion")
        return 2

    map_name, agents = load_scen(args.scen)
    grid = load_map(resolve_map(args.map, args.scen, map_name))
    n = args.agents if args.agents is not None else len(parsed.paths)
    if n > len(agents):
        print("ERROR: scenario has only %d agents" % len(agents))
        return 2
    res = validate_plan(grid, agents[:n], motion, parsed.paths,
                        max_errors=args.max_errors, solver_conflicts=parsed.solver_conflicts)
    for e in parsed.errors:
        res.n_errors += 1
        res.errors.insert(0, "parse: " + e)
    hdr = parsed.header
    if hdr and res.n_errors == 0 and (hdr[0] != n or hdr[1] != res.timesteps):
        res.n_errors += 1
        res.errors.append("header says agents=%d timesteps=%d" % hdr)
    if res.ok:
        if args.expect_mks is not None and res.mks != args.expect_mks:
            res.n_errors += 1
            res.errors.append("plan mks %d != expected %d" % (res.mks, args.expect_mks))
        if args.expect_soc is not None and res.soc != args.expect_soc:
            res.n_errors += 1
            res.errors.append("plan soc %d != expected %d" % (res.soc, args.expect_soc))
    if not res.ok:
        print("INVALID plan (%s motion): %d violation(s)" % (motion, res.n_errors))
        for e in res.errors[:args.max_errors]:
            print("  " + e)
        if res.n_errors > args.max_errors:
            print("  ...")
        return 1
    print("OK (%s motion) %s" % (motion, res.summary()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
