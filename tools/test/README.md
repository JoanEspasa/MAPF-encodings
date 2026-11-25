# Test harness for the MAPF SAT solver

Python 3, standard library only. All tools run every solver invocation in its own
fresh working directory, because the solver writes `tmp.out` and the `-c` CNF file
relative to the CWD. Monosat encodings get `./libs/monosat` (a symlink to
`libs/monosat`) in their job directory and an automatic `-c formula.cnf`.

`--binary` defaults to `release/MAPF`. While that binary is being rebuilt, point it
at a stable copy.

Files:

- `validate_plan.py`: checks a plan printed by `MAPF -p` (library and CLI).
- `oracle.py`: compares optimal costs against the published AAMAS2026 results.
- `fuzz.py`: differential fuzzer on random small grids.
- `solver_runner.py`: shared helper that runs the binary and parses the `-l 1` log line.

## validate_plan.py

```
MAPF -m MAPDIR -s X.scen -e soc_pebble_at_eager_single -a 8 -p > out.txt
python3 validate_plan.py -m MAPDIR -s X.scen [-a 8] [--motion pebble] out.txt   # or '-' for stdin
```

The validator checks that:

- each agent starts at its start and ends at its goal;
- every step waits or moves to a 4-adjacent free cell (vertex ids are free `.`
  cells numbered row-major);
- there are no vertex conflicts and no swap conflicts;
- under pebble motion, no agent moves into a vertex that another agent occupied
  at the previous timestep.

It also flags any `... conflict!` lines that the solver printed itself.

On success it prints the plan's mks and soc. An agent's cost is the first
timestep from which it stays at its goal, plus 1. mks is the largest agent cost
and soc is the sum of agent costs. The tool also prints the lower bounds
`max/sum(SP+1)`.

`--motion` defaults to the motion in the `Encoding used:` line, and `-m` can be a
map file or a map directory. Use `--expect-mks N` or `--expect-soc N` to also
require a specific cost. Exit codes: 0 means valid, 1 means violations (the first
ones are printed), 2 means usage or parse error.

## oracle.py

```
python3 oracle.py --check                      # self-consistency + table size
python3 oracle.py ours1.res ours2.res          # OK / MISMATCH / NO-ORACLE per row
python3 oracle.py --run --binary BIN --timeout 60 --jobs 4 \
    -i random_20_0:10,15,20 -i empty_40_0:20 [--objectives mks,soc] \
    [--encodings '{obj}_{motion}_at_eager_single'] [--workdir DIR]
```

The oracle table is built from every `*.res` under `experiments/AAMAS2026`.

- The table key is (scenario basename, agents, objective, motion). The value is
  column 7 for mks and column 8 for soc.
- A row is used only if its last field is `sat` and its solver_calls (column 11)
  is greater than 0. `results_kissat/results_50agents.res` is a build-only run:
  0 solver calls and the lower bound as the "solution", yet still marked `sat`.
- All published rows use parallel motion, so pebble rows of ours get `NO-ORACLE`.
- Our lower bounds (columns 5 and 6) are compared with the published ones as well.
  A lower-bound difference means the instance files differ, and it counts as a
  MISMATCH. The paper's instances are in
  `experiments/AAMAS2026/MAPF_Encodings/instances`. The repo's `instances/`
  directory uses the same names but holds different contents.

`--run` solves paper instances with `-p`, validates every plan, writes the combined
log to `<workdir>/results.res` and compares it against the oracle. The exit code
is non-zero if there is any MISMATCH or an invalid plan, and for `--check` if the
published results disagree with each other.

## fuzz.py

```
python3 fuzz.py --binary BIN --cases 150 --seed 1 --jobs 8 --timeout 20 \
    --objectives mks,soc --motions parallel,pebble \
    --reference '{obj}_{motion}_at_eager_single' \
    --candidates '{obj}_{motion}_pass_eager_single,{obj}_{motion}_shift_eager_single,{obj}_{motion}_at_lazy_single' \
    [--candidate '{obj}_{motion}_at_eager_dupli:-d 0'] [--workdir DIR] [--keep] [-v]
```

Instances:

- grids are 4x4 to 8x8 with 0-30% obstacles, and only the largest 4-connected
  component is kept free;
- each instance has 2-10 agents with distinct starts and distinct goals;
- with probability `--stay-prob`, an agent's goal equals its start.

Each instance runs for every (objective, motion). For each one, the reference
runs first. If it times out, the task is SKIPPED. Either the instance is unsolvable, or
the SoC formula build is too slow. The report marks skipped tasks that are
known to be solvable because another objective or motion solved them.
Otherwise every candidate runs with `--cand-timeout`, which defaults to twice
`--timeout`. Failure kinds:

- `invalid-plan`: the plan fails validation.
- `plan-cost`: the cost recomputed from the plan differs from the log line.
- `lb`: the log's lower bounds differ from BFS.
- `cost-mismatch`: a candidate's optimum differs from the reference.
- `timeout` or `error`: a candidate failed to solve.
- `ref-*`: the same problems, found in the reference itself.

Monosat templates expanded to pebble or lazy are skipped as unsupported.

Failing instances are kept: `<workdir>/<case>/<case>.map|.scen`, with one
directory per run holding `cmd.sh`, `stdout.txt` and `result.res`. The summary
(`report.txt`, `failures.json`) gives a copy-paste reproduction command. Use
`--case-ids 12,40` with the same `--seed` to regenerate and re-run only those
cases. Passing cases are deleted unless you pass `--keep`. The exit code is
non-zero on any failure.
