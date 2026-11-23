"""Navigation replanning. Cell IDs in change journals are y * width + x."""
import hashlib
import json
import math
import random
import time
from functools import lru_cache

from .data import DATA
from .engine import Engine

CONTROLS = DATA / "base_control_set.txt"


def seed_for(*parts):
    payload = json.dumps(parts, separators=(",", ":"), ensure_ascii=True).encode()
    return int.from_bytes(hashlib.sha256(payload).digest()[:8], "big")


@lru_cache(maxsize=2)
def primitives(filename=CONTROLS):
    controls = {}
    for section in filename.read_text().split("===== prim description: =====")[1:]:
        fields = dict(line.split(": ", 1) for line in section.splitlines() if ": " in line)
        dy, dx, end = map(int, fields["goal state (i, j, heading num)"].split())
        trace = [tuple(map(int, row.split()))[::-1] for row in
                 section.split("collision is:")[1].split("---")[0].strip().splitlines()]
        ident = int(fields["prim ID is"])
        if ident in controls:
            raise ValueError("Duplicate primitive ID")
        controls[ident] = (dx, dy, int(fields["start heading (number)"]), end,
                           max(float(fields["length is"]), math.hypot(dx, dy)), trace)
    return controls


def read_grid(path):
    _, height, width, _, *rows = path.read_text().splitlines()
    w, h = int(width.split()[1]), int(height.split()[1])
    if len(rows) != h or any(len(row) != w for row in rows):
        raise ValueError(f"Malformed map {path}")
    return w, bytearray(c not in ".GS" for row in rows for c in row)


def changes(grid, percent, protected, seed):
    count = math.floor(len(grid) * percent / 200 + 0.5)
    if count == 0:
        return [], []
    rng = random.Random(seed)
    order = list(range(len(grid)))
    rng.shuffle(order)
    block, clear = [], []
    for cell in order:
        if cell in protected:
            continue
        side = clear if grid[cell] else block
        if len(side) < count:
            side.append(cell)
        if len(block) == len(clear) == count:
            return block, clear
    raise ValueError(f"Cannot exchange {count} pairs of eligible cells")


def write_record(stream, record):
    stream.write(json.dumps(record, separators=(",", ":")) + "\n")
    stream.flush()


def navigate(spec, stream, deadline=None, recorded=None, *, engine_factory=None, on_move=None):
    task = spec["task"]
    percent, seed = spec["percent"], spec["seed"]
    map_file = DATA / (task["map"] + ".map")
    width, grid = read_grid(map_file)
    controls = primitives()
    goal, pose = tuple(task["goal"]), tuple(task["start"])
    moved = event = 0
    nav_ms = first_ms = replan_ms = 0.0
    write_record(stream, {"kind": "episode", **spec})
    factory = engine_factory or Engine
    with factory(map_file, CONTROLS, spec["graph"], spec["algorithm"], goal, deadline) as engine:
        init_ms = engine.receive()["init_ms"]
        while True:
            t = time.perf_counter()
            block, clear = [], []
            manual = spec.get("edits", {}).get(moved)
            if manual:
                for c, value in manual.items():
                    if value and c in (pose[1]*width+pose[0], goal[1]*width+goal[0]):
                        raise ValueError("The robot and goal must remain free")
                    if grid[c] != value:
                        (block if value else clear).append(c)
                        grid[c] = value
            elif event:
                cell = lambda p: p[1] * width + p[0]
                if recorded is None:
                    block, clear = changes(grid, percent, {cell(goal), cell(pose)},
                                           seed_for(seed, percent, event))
                else:
                    expected = recorded[event]
                    if list(pose) != expected["pose"] or moved != expected["moved"]:
                        raise ValueError(f"Robot position differs at event {event}")
                    block, clear = expected["block"], expected["clear"]
                    count = math.floor(len(grid) * percent / 200 + 0.5)
                    if len(block) != count or len(clear) != count:
                        raise ValueError(f"Wrong change count at event {event}")
                for c in block:
                    grid[c] = 1
                for c in clear:
                    grid[c] = 0
            nav_ms += (time.perf_counter() - t) * 1000
            request = {"kind": "request", "event": event, "pose": pose, "moved": moved,
                       "block": block, "clear": clear}
            write_record(stream, request)
            answer = engine.query(pose, block, clear)
            if event:
                replan_ms += answer["plan_ms"]
            else:
                first_ms = answer["plan_ms"]
            write_record(stream, {"kind": "result", "event": event, **answer})
            t = time.perf_counter()
            if answer["outcome"] == "no_path":
                status = "no_path"
                break
            for ident in answer["route"][:10]:
                dx, dy, start_h, end_h, _, _ = controls[ident]
                if pose[2] != start_h:
                    raise ValueError("Incorrect planned heading")
                pose = (pose[0]+dx, pose[1]+dy, end_h)
                moved += 1
                if on_move:
                    on_move(ident, pose)
                if moved in spec.get("edits", {}):
                    break
            if pose == goal:
                status = "reached"
                break
            if len(answer["route"]) <= 10 and moved not in spec.get("edits", {}):
                raise ValueError("Incomplete route")
            nav_ms += (time.perf_counter()-t)*1000
            event += 1
        nav_ms += (time.perf_counter()-t)*1000
    summary = {"status": status, "moves": moved, "events": event, "first_ms": first_ms,
               "replan_ms": replan_ms, "init_ms": init_ms, "navigation_ms": nav_ms,
               "total_ms": init_ms + first_ms + replan_ms + nav_ms}
    write_record(stream, {"kind": "summary", **summary})
    return summary

