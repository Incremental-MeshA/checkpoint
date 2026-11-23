import argparse
import csv
import io
import itertools
import json
import math
import os
import platform
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path

from .data import DATA, ROOT, LOCK, digest, prepare
from .engine import executable
from .navigation import navigate, seed_for, write_record
from .select import MANIFEST, select
from .visibility import INITIAL_PERCENT, PERIOD, navigate_visible

ALGORITHMS = ["astar", "dstar_lite", "mpgaa"]
GRAPHS = ["lattice", "mesh"]
VARIANTS = list(itertools.product(ALGORITHMS, GRAPHS)) + [
    ("dstar_lite_chain", "mesh"), ("mpgaa_chain", "mesh"), ("mpgaa_lazy", "mesh")
]
SCENARIOS = {"navigation_replanning": navigate, "limited_visibility": navigate_visible}
SUMMARY = ["task", "map", "algorithm", "graph", "percent", "repeat", "seed", "status",
           "scenario", "radius", "moves", "events", "world_updates", "total_ms", "init_ms", "first_ms", "replan_ms", "navigation_ms",
           "expanded", "expanded_cells_sum", "open_ops", "repair_ops", "journal"]


def cpu():
    if os.name == "nt":
        import winreg
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DESCRIPTION\System\CentralProcessor\0") as key:
            return winreg.QueryValueEx(key, "ProcessorNameString")[0].strip()
    info = Path("/proc/cpuinfo")
    if info.exists():
        for row in info.read_text().splitlines():
            if row.startswith("model name"):
                return row.split(":", 1)[1].strip()
    return platform.processor()


def provenance():
    binary = executable()
    build_settings = list(binary.parent.glob("build-Release.txt"))
    if not build_settings:
        build_settings = list(binary.parent.parent.glob("build-Release.txt"))
    return {"binary_sha256": digest(binary.read_bytes()), "machine": platform.node(), "cpu": cpu(),
            "os": platform.platform(), "python": sys.version,
            "build": build_settings[0].read_text().splitlines() if build_settings else ["external binary"],
            "compiler_env": {"CXXFLAGS": os.environ.get("CXXFLAGS", "")},
            "sources": json.loads(LOCK.read_text()),
            "manifest_sha256": digest(MANIFEST.read_bytes())}


def read_log(path):
    return [json.loads(row) for row in path.read_text().splitlines()]


def clean(records):
    return [{k: v for k, v in record.items() if not k.endswith("_ms")} for record in records]


def run(args):
    config = json.loads(args.config.read_text())
    scenario = config.get("scenario", "navigation_replanning")
    simulate = SCENARIOS[scenario]
    params = {}
    if scenario == "limited_visibility":
        radius = args.radius if args.radius is not None else config["radius"]
        if "radii" in config and radius not in config["radii"]:
            raise ValueError(f"Choose a visibility radius from {config['radii']}")
        params["radius"] = radius
    elif args.radius is not None:
        raise ValueError("--radius requires the limited_visibility scenario")
    profile = config["profiles"][args.profile]
    seconds = profile["budget_seconds"] if args.budget_seconds is None else args.budget_seconds
    percent = float(config["change_percent"] if args.change_percent is None else args.change_percent)
    repeats = args.repeats if args.repeats is not None else profile["repeats"]
    if not math.isfinite(percent) or not 0 <= percent <= 100 or not math.isfinite(seconds) or seconds <= 0 or repeats < 1:
        raise ValueError("Invalid percent, budget or repeats")
    if scenario == "navigation_replanning" and "change_percents" in config and percent not in config["change_percents"]:
        raise ValueError(f"Choose a change percentage from {config['change_percents']}")
    episode_seconds = args.episode_seconds if args.episode_seconds is not None else config.get("episode_seconds")
    if episode_seconds is not None and (not math.isfinite(episode_seconds) or episode_seconds <= 0):
        raise ValueError("Episode timeout must be a positive finite number")
    # Verify the published lock locally. Missing data must be prepared explicitly.
    sources = json.loads(LOCK.read_text())
    for name, item in sources.items():
        if digest((DATA/name).read_bytes()) != item["sha256"]:
            raise ValueError(f"Source hash mismatch: {name}")
    tasks = [t for t in json.loads(MANIFEST.read_text())["tasks"]
             if (t["id"] in profile["pairs"] if "pairs" in profile else args.profile != "quick" or t["quick"])
             and t["map"] in config["maps"]
             and (not config["pairs"] or t["id"] in config["pairs"])]
    if not tasks:
        raise ValueError("The profile selected no tasks")
    out = args.output or ROOT / "runs" / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    variants = [v for v in VARIANTS if not args.algorithm or v[0] == args.algorithm]
    out.mkdir(parents=True, exist_ok=False)
    metadata = {**provenance(), "profile": args.profile, "percent": percent, "budget_seconds": seconds,
                "config": config, "variants": variants, "repeats": repeats, "tasks": tasks}
    if episode_seconds is not None:
        metadata["episode_budget_seconds"] = episode_seconds
    if params:
        metadata.update(**params, initial_free_percent=INITIAL_PERCENT, world_update_period=PERIOD)
    (out/"run.json").write_text(json.dumps(metadata, indent=2))
    started = time.monotonic()
    deadline = started + seconds
    counts = {}
    with (out/"summary.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, SUMMARY)
        writer.writeheader()
        for repeat, task, (algo, graph) in itertools.product(range(repeats), tasks, variants):
            seed = seed_for(config["seed"], task["id"], repeat)
            name = f"{task['id']}-{algo}-{graph}-r{repeat}.jsonl"
            row = dict(task=task["id"], map=task["map"], algorithm=algo, graph=graph,
                       percent=percent, repeat=repeat, seed=seed, status="not_run", scenario=scenario, **params)
            if time.monotonic() < deadline:
                spec = dict(task=task, algorithm=algo, graph=graph, percent=percent, seed=seed, scenario=scenario, **params)
                journal = out/name
                row["journal"] = name
                with journal.open("w") as stream:
                    try:
                        episode_deadline = deadline
                        if episode_seconds is not None:
                            episode_deadline = min(deadline, time.monotonic() + episode_seconds)
                        row.update(simulate(spec, stream, episode_deadline))
                    except (TimeoutError, RuntimeError, ValueError, OSError) as err:
                        row["status"] = "timeout" if isinstance(err, TimeoutError) else "error"
                        reason = str(err)
                        if isinstance(err, TimeoutError) and episode_deadline < deadline:
                            reason = f"Episode exceeded {episode_seconds:g} seconds"
                        write_record(stream, {"kind": "interrupted", "status": row["status"], "error": reason})
                results = [r for r in read_log(journal) if r["kind"] == "result"]
                row.update(expanded=sum(r["expanded"] for r in results),
                           expanded_cells_sum=sum(r["expanded_cells"] for r in results),
                           open_ops=sum(sum(r["open"].values()) for r in results),
                           repair_ops=sum(sum(r["repair_queue"].values()) for r in results))
            writer.writerow(row)
            f.flush()
            counts[row["status"]] = counts.get(row["status"], 0) + 1
    metadata.update(wall_seconds=time.monotonic()-started, outcomes=counts)
    (out/"run.json").write_text(json.dumps(metadata, indent=2)+"\n")
    print(out, counts, f"{metadata['wall_seconds']:.3f}s", flush=True)
    return int(any(status in counts for status in ("timeout", "error", "not_run")))


def replay(path, regenerate=False):
    if path.is_dir():
        with (path/"summary.csv").open(newline="") as stream:
            completed = [r for r in csv.DictReader(stream) if r["status"] in ("reached","no_path")]
        if not completed:
            raise ValueError("No completed episodes in directory")
        for row in completed:
            replay(path/row["journal"], regenerate)
        return
    original = read_log(path)
    if not original or original[-1]["kind"] != "summary":
        raise ValueError("Replay needs a completed episode")
    spec = {k: v for k, v in original[0].items() if k != "kind"}
    requests = [r for r in original if r["kind"] == "request"]
    output = io.StringIO()
    scenario = spec.get("scenario", "navigation_replanning")
    edits = [r for r in original if r["kind"] == "world"] if scenario == "limited_visibility" else requests
    SCENARIOS[scenario](spec, output, recorded=None if regenerate else edits)
    actual = [json.loads(line) for line in output.getvalue().splitlines()]
    # Older journals predate the per-batch update counter.
    for saved, current in zip(original, actual):
        if saved["kind"] == "result" and "batch_seeds" not in saved:
            current.pop("batch_seeds", None)
    if clean(original) != clean(actual):
        raise ValueError(f"Replay mismatch in {path}")
    print(f"Verified {len(requests)} requests: {path.name}")


def compare(path):
    with (path/"summary.csv").open(newline="") as f:
        rows = list(csv.DictReader(f))
    # Matched task/repetition cohorts, including navigation failures in outcome counts.
    by_pair = {}
    for row in rows:
        by_pair.setdefault((row["task"], row["repeat"]), []).append(row)
    variants = list(dict.fromkeys((r["algorithm"], r["graph"]) for r in rows))
    paired = [r for group in by_pair.values() if len(group) == len(variants) and
              all(r["status"] in ("reached", "no_path") for r in group) for r in group]
    visible = bool(rows) and all(row.get("scenario") == "limited_visibility" for row in rows)
    writer = csv.writer(sys.stdout, lineterminator="\n")
    writer.writerow(["algorithm", "graph", "paired", "reached", "no_path", "events",
                     "median_total_ms", "median_replan_sum_ms", "expansions", "expanded_cells_sum", "open_ops", "repair_ops"]
                    + (["radius", "world_updates"] if visible else []))
    for algo, graph in variants:
        group = [r for r in paired if (r["algorithm"],r["graph"]) == (algo,graph)]
        median = lambda key: round(statistics.median(float(r[key]) for r in group), 3) if group else ""
        total = lambda key: sum(int(r[key]) for r in group)
        writer.writerow([algo, graph, len(group), sum(r["status"]=="reached" for r in group),
                         sum(r["status"]=="no_path" for r in group), total("events"), median("total_ms"),
                         median("replan_ms"), total("expanded"), total("expanded_cells_sum"), total("open_ops"), total("repair_ops")]
                        + ([rows[0]["radius"], total("world_updates")] if visible else []))


def main():
    parser = argparse.ArgumentParser(description="Motion primitive replanning benchmark")
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    prep.add_argument("--select", action="store_true", help="Regenerate the fixed manifest with lattice A*")
    runner = sub.add_parser("run")
    runner.add_argument("--config", type=Path, default=ROOT/"config"/"benchmark.json")
    runner.add_argument("--profile", choices=["quick","long"], default="quick")
    runner.add_argument("--algorithm", choices=sorted({a for a, _ in VARIANTS}),
                        help="Run only this algorithm on its supported graphs")
    runner.add_argument("--change-percent", type=float)
    runner.add_argument("--radius", type=int, help="Sensor radius in cells from the scenario configuration")
    runner.add_argument("--budget-seconds", type=float)
    runner.add_argument("--episode-seconds", type=float, help="Wall-time limit for each episode")
    runner.add_argument("--repeats", type=int)
    runner.add_argument("--output", type=Path)
    player = sub.add_parser("replay")
    player.add_argument("journal", type=Path)
    player.add_argument("--regenerate", action="store_true", help="Regenerate all map changes from the episode seed")
    sub.add_parser("compare").add_argument("directory", type=Path)
    viewer = sub.add_parser("visualize", help="Open the interactive car16 episode viewer")
    viewer.add_argument("--port", type=int, default=0, help="Local HTTP port (default: automatic)")
    viewer.add_argument("--no-browser", action="store_true", help="Print the URL without opening a tab")
    args = parser.parse_args()
    if args.command == "prepare":
        prepare()
        if args.select or not MANIFEST.exists():
            select()
    elif args.command == "run":
        return run(args)
    elif args.command == "replay":
        replay(args.journal, args.regenerate)
    elif args.command == "visualize":
        from .viewer import serve
        serve(args.port, not args.no_browser)
    else:
        compare(args.directory)
    return 0


if __name__ == "__main__":
    sys.exit(main())







