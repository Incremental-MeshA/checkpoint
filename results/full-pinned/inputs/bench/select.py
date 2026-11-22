"""Deterministic stratified selection, followed by an exact lattice feasibility check."""
import json
from .data import DATA, MAPS, RANDOM_MAPS, ROOT
from .engine import Engine
from .navigation import CONTROLS, seed_for

MANIFEST = ROOT / "config" / "tasks.json"


def select(seed=20261122):
    tasks = []
    # Distinct distance bands. Quick uses two rotating bands per map.
    bands = [(42, 62), (65, 87), (90, 115)]
    for index, name in enumerate(MAPS):
        rows = (DATA / (name + ".map.scen")).read_text().splitlines()
        for group in range(3):
            band = (index + group) % len(bands)
            lo, hi = bands[band]
            candidates = [(line, row.split()) for line, row in enumerate(rows[1:], 2)
                          if lo <= float(row.split()[-1]) < hi]
            candidates.sort(key=lambda item: seed_for(seed, name, item[0]))
            for line, fields in candidates:
                pose_seed = seed_for(seed, name, line)
                start = [int(fields[4]), int(fields[5]), pose_seed % 16]
                goal = [int(fields[6]), int(fields[7]), (pose_seed >> 4) % 16]
                with Engine(DATA / (name+".map"), CONTROLS, "lattice", "astar", goal) as engine:
                    engine.receive()
                    answer = engine.query(start)
                min_moves = 20 if name not in RANDOM_MAPS and band > 0 else 10
                if answer["outcome"] == "found" and len(answer["route"]) > min_moves:
                    tasks.append({"id": f"{name}-L{line}", "map": name, "line": line,
                                  "start": start, "goal": goal, "pose_seed": pose_seed,
                                  "grid_length": float(fields[-1]), "band": ["short","medium","long"][band],
                                  "initial_cost": answer["cost"], "initial_moves": len(answer["route"]),
                                  "quick": group < 2})
                    break
            else:
                raise ValueError(f"No eligible pair in {name} for band {band}")
    MANIFEST.write_text(json.dumps({"selection_seed": seed, "tasks": tasks}, indent=2)+"\n")
    return tasks


