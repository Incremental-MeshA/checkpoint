"""Replay one mesh control trajectory per map/condition on all five planners."""
import argparse
import csv
import json
import time
from pathlib import Path

from .data import DATA, digest
from .engine import Engine, executable
from .navigation import CONTROLS
from .prefix_report import COMPLETE, VARIANTS, write_csv


def replay(campaign, output):
    rows = []
    binary = digest(executable().read_bytes())
    for manifest in sorted(campaign.glob("*/run.json")):
        meta = json.loads(manifest.read_text())
        if meta["binary_sha256"] != binary:
            raise ValueError("Set MESH_PLANNER to the recorded campaign binary")
        with (manifest.parent/"summary.csv").open(newline="") as stream:
            controls = [r for r in csv.DictReader(stream) if
                        (r["algorithm"],r["graph"],r["repeat"]) == ("mpgaa","mesh","0") and
                        r["status"] in COMPLETE and int(r["events"]) > 0]
        for map_name in meta["config"]["maps"]:
            # Fixed selection in manifest order, independent of candidate times.
            row = next(r for r in controls if r["map"] == map_name)
            log = (manifest.parent/row["journal"]).read_text().splitlines()
            task = json.loads(log[0])["task"]
            requests, expected = [], []
            for line in log[1:]:
                record = json.loads(line)
                if record["kind"] == "request":
                    requests.append(record)
                elif record["kind"] == "result":
                    expected.append((record["outcome"], record["cost"]))
            assert len(expected) == len(requests)
            for algorithm, graph in VARIANTS:
                verified = 0
                replan_ms = 0.
                try:
                    with Engine(DATA/(map_name+".map"), CONTROLS, graph, algorithm, task["goal"],
                                deadline=time.monotonic()+60) as engine:
                        engine.receive()
                        for i, req in enumerate(requests):
                            res = engine.query(req["pose"],req["block"],req["clear"])
                            if (res["outcome"],res["cost"]) != expected[i]:
                                raise ValueError(f"{algorithm}/{graph} differs at {row['journal']} request {i}")
                            if i:
                                replan_ms += res["plan_ms"]
                            verified += 1
                    status = "verified"
                except TimeoutError:
                    status = "timeout"
                rows.append(dict(series=manifest.parent.name, task=task["id"], map=map_name,
                                 repeat=0, algorithm=algorithm, graph=graph, requests=len(requests),
                                 verified=verified, status=status, replan_ms=replan_ms))
    write_csv(output,rows)


if __name__ == "__main__":
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument("campaign",type=Path)
    cli.add_argument("output",type=Path)
    args=cli.parse_args()
    replay(args.campaign,args.output)
