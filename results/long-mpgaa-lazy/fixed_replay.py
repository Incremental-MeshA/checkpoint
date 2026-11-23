"""Compare planners on the exact request streams driven by mpgaa_lazy in this run."""
import csv
import hashlib
import json
import statistics
import sys
import time
from pathlib import Path

OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(OUT.parents[1]))
from bench.data import DATA
from bench.engine import Engine, executable
from bench.navigation import CONTROLS, primitives

METHODS = [("mpgaa_lazy", "mesh"), ("mpgaa", "mesh"), ("astar", "mesh"),
           ("mpgaa", "lattice"), ("mpgaa_chain", "mesh")]
SERIES = [f"navigation-{v}" for v in ("0.1", "0.5", "1", "5")] + [
    f"visibility-{r}" for r in (25, 50, 100, 200)]
UNIT = 1 << 52
WEIGHTS = {ident: (int(motion[4]*UNIT), len(motion[5])-1)
           for ident, motion in primitives().items()}


def cost(answer):
    if answer["outcome"] == "no_path":
        assert answer["cost"] is None and not answer["route"]
        return None
    assert answer["outcome"] == "found"
    weights = [WEIGHTS[ident] for ident in answer["route"]]
    length = sum(w[0] for w in weights)
    assert length/UNIT == answer["cost"]
    return length, sum(w[1] for w in weights)


def replay(row, folder, rotation):
    with (folder/row["journal"]).open() as stream:
        records = [json.loads(line) for line in stream]
    task = records[0]["task"]
    queries = [r for r in records if r["kind"] == "request"]
    answers = [r for r in records if r["kind"] == "result"]
    assert len(queries) == len(answers) == int(row["events"])+1
    reference = [cost(r) for r in answers]
    for algorithm, graph in METHODS[rotation:] + METHODS[:rotation]:
        stats = dict(series=folder.name, task=row["task"], repeat=row["repeat"],
                     algorithm=algorithm, graph=graph, requests=len(queries),
                     status="complete", checked=0, reachability_diff=0, length_diff=0,
                     mesh_steps_diff=0, init_ms=0, first_ms=0, replan_ms=0)
        try:
            with Engine(DATA/(task["map"]+".map"), CONTROLS, graph, algorithm,
                        task["goal"], time.monotonic()+60) as engine:
                stats["init_ms"] = engine.receive()["init_ms"]
                for i, q in enumerate(queries):
                    r = engine.query(q["pose"], q["block"], q["clear"])
                    result, expected = cost(r), reference[i]
                    stats["checked"] += 1
                    if (result is None) != (expected is None):
                        stats["reachability_diff"] += 1
                    elif result is not None:
                        stats["length_diff"] += result[0] != expected[0]
                        if graph == "mesh" and algorithm != "astar":
                            stats["mesh_steps_diff"] += result[1] != expected[1]
                    if algorithm == "mpgaa_lazy":
                        assert r["route"] == answers[i]["route"]
                    stats["replan_ms" if i else "first_ms"] += r["plan_ms"]
        except TimeoutError:
            stats["status"] = "timeout"
        yield stats


def write_csv(path, rows):
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, rows[0])
        writer.writeheader()
        writer.writerows(rows)


def main():
    digest = hashlib.sha256(executable().read_bytes()).hexdigest()
    timings, summary, navigation = [], [], []
    for series in SERIES:
        folder = OUT/series
        meta = json.loads((folder/"run.json").read_text())
        assert digest == meta["binary_sha256"]
        with (folder/"summary.csv").open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) == len(meta["tasks"])*meta["repeats"] == 54
        assert meta["profile"] == "long" and meta["variants"] == [["mpgaa_lazy", "mesh"]]
        navigation.extend(dict(series=series, **{**row, "journal": f"{series}/{row['journal']}"})
                          for row in rows)
        group = []
        for index, row in enumerate(rows):
            if row["status"] in ("reached", "no_path"):
                group.extend(replay(row, folder, index % len(METHODS)))
        timings.extend(group)
        # Include every completed stream, also when its final answer was no_path.
        timeout = {(r["task"], r["repeat"]) for r in group if r["status"] == "timeout"}
        for algorithm, graph in METHODS:
            measured = [r for r in group if (r["algorithm"], r["graph"]) == (algorithm, graph)]
            paired = [r for r in measured if (r["task"], r["repeat"]) not in timeout]
            def median(field):
                return statistics.median(r[field] for r in paired) if paired else ""
            summary.append(dict(series=series, algorithm=algorithm, graph=graph,
                                paired=len(paired), checked=sum(r["checked"] for r in measured),
                                timeout=sum(r["status"] == "timeout" for r in measured),
                                reachability_diff=sum(r["reachability_diff"] for r in measured),
                                length_diff=sum(r["length_diff"] for r in measured),
                                mesh_steps_diff=sum(r["mesh_steps_diff"] for r in measured),
                                median_init_ms=median("init_ms"), median_first_ms=median("first_ms"),
                                median_replan_ms=median("replan_ms")))
        write_csv(OUT/"fixed-replay.csv", timings)
        write_csv(OUT/"comparison.csv", summary)
        write_csv(OUT/"summary.csv", navigation)
        print(series, "verified", len(rows), "episode records", flush=True)


if __name__ == "__main__":
    main()
