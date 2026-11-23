"""Rebuild and check the long campaign summary using its recorded manifests and journals."""
import csv
import hashlib
import json
import os
import sys
from collections import Counter
from pathlib import Path

out = Path(__file__).resolve().parent
sys.path.insert(0, str(out.parents[1]))
from bench.__main__ import replay
from bench.visualization import render


def write_csv(name, rows):
    with (out/name).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def summarize():
    rows, series = [], []
    samples = {}
    total_requests = 0
    binary = out/"bin/planner.exe"
    os.environ["MESH_PLANNER"] = str(binary)
    digest = hashlib.sha256(binary.read_bytes()).hexdigest()
    for cfg_name, prefix, param, values in (("benchmark", "navigation", "percent", "change_percents"),
                                           ("visibility", "visibility", "radius", "radii")):
        config = json.loads((out/(cfg_name+".json")).read_text())
        for value in config[values]:
            tag = f"{prefix}-{value:g}"
            folder = out/tag
            meta = json.loads((folder/"run.json").read_text())
            assert meta["profile"] == "long" and meta["config"] == config
            assert meta[param] == value and meta["repeats"] == 3
            assert meta["binary_sha256"] == digest and meta["episode_budget_seconds"] == 60
            with (folder/"summary.csv").open(newline="") as stream:
                group = list(csv.DictReader(stream))
            expected = {(t["id"], r, a, g) for t in meta["tasks"] for r in range(meta["repeats"])
                        for a,g in meta["variants"]}
            assert len(expected) == len(group) == 432
            assert expected == {(v["task"],int(v["repeat"]),v["algorithm"],v["graph"]) for v in group}
            outcomes = Counter(v["status"] for v in group)
            assert dict(outcomes) == meta["outcomes"]
            failed = set()
            requests = replans = 0
            non_quick = {t["id"] for t in meta["tasks"] if not t["quick"]}
            for record in group:
                row = dict(series=tag, **record)
                status = row["status"]
                if status not in ("reached","no_path"):
                    failed.add((row["task"],row["repeat"]))
                if row["journal"]:
                    row["journal"] = tag+"/"+row["journal"]
                    nq = nr = 0
                    with (out/row["journal"]).open() as stream:
                        header = json.loads(next(stream))
                        assert header["task"]["id"] == row["task"] and header["seed"] == int(row["seed"])
                        for line in stream:
                            last = json.loads(line)
                            if last["kind"] == "request": nq += 1
                            elif last["kind"] == "result": nr += 1
                    requests += nq
                    replans += max(nq-1, 0)
                    if status in ("reached", "no_path"):
                        assert last["kind"] == "summary" and last["status"] == status
                        assert nq == nr == int(row["events"]) + 1
                        if nq > 1 and row["task"] in non_quick and int(row["repeat"]) > 0:
                            key = cfg_name, row["algorithm"], row["graph"]
                            if key not in samples or float(row["total_ms"]) < float(samples[key]["total_ms"]):
                                samples[key] = row
                    else:
                        assert last["kind"] == "interrupted" and last["status"] == status
                else:
                    assert status == "not_run"
                rows.append(row)
            series.append(dict(series=tag, episodes=len(group), matched_task_repeats=54-len(failed),
                               **{s:outcomes[s] for s in ("reached","no_path","timeout","error","not_run")},
                               requests=requests, replan_requests=replans, wall_seconds=meta["wall_seconds"]))
            total_requests += requests
    write_csv("summary.csv", rows)
    write_csv("outcomes.csv", series)
    render(out, out/"timing.html")
    for row in samples.values():
        replay(out/row["journal"], regenerate=True)
    audit = dict(episodes=len(rows), planning_requests=total_requests,
                 outcomes=dict(Counter(r["status"] for r in rows)),
                 replans=sum(s["replan_requests"] for s in series),
                 wall_seconds=sum(s["wall_seconds"] for s in series),
                 regenerated=[r["journal"] for r in samples.values()])
    (out/"validation.json").write_text(json.dumps(audit,indent=2)+"\n")
    print(audit)


if __name__ == "__main__":
    summarize()

