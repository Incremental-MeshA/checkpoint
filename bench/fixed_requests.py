"""Run the experiments cohort on identical saved MPGAA* queries (CSV to stdout)."""
import argparse
import csv
import math
import sys
from pathlib import Path

from .__main__ import RESEARCH_SUITE, read_log
from .data import DATA
from .engine import Engine
from .navigation import CONTROLS


def compare_requests(directory, repeat):
    with (directory / "summary.csv").open(newline="") as stream:
        rows = [r for r in csv.DictReader(stream) if r["algorithm"] == "mpgaa" and
                r["graph"] == "mesh" and int(r["repeat"]) == repeat]
    if not rows:
        raise ValueError(f"No reference episodes for repeat {repeat} in {directory}")
    for row in rows:
        if row["status"] not in ("no_path", "reached"):
            raise ValueError(f"Incomplete reference episode in {directory}: {row['task']}")
        records = read_log(directory / row["journal"])
        goal = records[0]["task"]["goal"]
        requests = [r for r in records if r["kind"] == "request"]
        reference = [r for r in records if r["kind"] == "result"]
        if not requests or len(requests) != len(reference):
            raise ValueError(f"Incomplete request log: {row['journal']}")
        for algorithm, graph in RESEARCH_SUITE:
            with Engine(DATA / (row["map"] + ".map"), CONTROLS, graph, algorithm, goal) as engine:
                engine.receive()
                for request, expected in zip(requests, reference):
                    actual = engine.query(request["pose"], request["block"], request["clear"])
                    cost = actual["cost"]
                    match = actual["outcome"] == expected["outcome"] and (
                        cost == expected["cost"] or
                        (cost is not None and expected["cost"] is not None and
                         math.isclose(cost, expected["cost"], rel_tol=0, abs_tol=1e-6)))
                    yield (directory.name, row["task"], repeat, row["seed"], request["event"],
                           algorithm, actual["outcome"], cost, expected["cost"], int(match),
                           actual["plan_ms"], actual["expanded"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directories", nargs="+", type=Path)
    parser.add_argument("--repeat", type=int, default=0)
    args = parser.parse_args()
    writer = csv.writer(sys.stdout, lineterminator="\n")
    writer.writerow(("series", "task", "repeat", "seed", "event", "algorithm", "outcome",
                     "cost", "reference_cost", "match", "plan_ms", "expanded"))
    matched = True
    for directory in args.directories:
        for row in compare_requests(directory, args.repeat):
            writer.writerow(row)
            matched &= bool(row[9])
    return int(not matched)


if __name__ == "__main__":
    sys.exit(main())
