"""Navigation with a remembered map and disk-shaped local observations."""
import math
import random
import time
from collections import deque

from .data import DATA
from .engine import Engine
from .navigation import CONTROLS, primitives, read_grid, seed_for, write_record

INITIAL_PERCENT = 5
PERIOD = 20


def world_change(grid, protected, seed, number, percent, recorded=None):
    """Both sides are sampled from the same pre-event snapshot."""
    free_count = grid.count(0)
    rate = percent if number else INITIAL_PERCENT
    count = math.floor(free_count * rate / 100 + 0.5)
    clear_count = count if number else 0
    if recorded is None:
        rng = random.Random(seed_for(seed, "visibility-world", number))
        def sample(value, size):
            if not size:
                return []
            pool = [c for c, state in enumerate(grid) if state == value]
            # The same candidate order applies even if robots chose different poses.
            candidates = rng.sample(pool, min(len(pool), size + len(protected)))
            return [c for c in candidates if c not in protected][:size]
        block, clear = sample(0, count), sample(1, clear_count)
    else:
        block, clear = recorded["block"], recorded["clear"]
    if len(block) != count or len(clear) != clear_count:
        raise ValueError("Insufficient eligible cells or incorrect recorded world change")
    for cells, value in ((block, 1), (clear, 0)):
        if (len(set(cells)) != len(cells) or any(c in protected or c < 0 or c >= len(grid)
                or grid[c] == value for c in cells)):
            raise ValueError("Invalid world change")
    for c in block:
        grid[c] = 1
    for c in clear:
        grid[c] = 0
    return block, clear


def observe(actual, known, width, pose, radius):
    """Update only cells whose centres lie inside the closed Euclidean disk."""
    x, y, _ = pose
    block, clear = [], []
    height = len(actual) // width
    for row in range(max(0, y-radius), min(height, y+radius+1)):
        half = math.isqrt(radius*radius - (row-y)**2)
        left = row*width + max(0, x-half)
        right = row*width + min(width, x+half+1)
        latest = actual[left:right]
        previous = known[left:right]
        if latest == previous:
            continue
        for i, (old, new) in enumerate(zip(previous, latest), left):
            if old != new:
                (block if new else clear).append(i)
        known[left:right] = latest
    return block, clear


def navigate_visible(spec, stream, deadline=None, recorded=None, *, engine_factory=None, on_move=None):
    task = spec["task"]
    pose, goal = tuple(task["start"]), tuple(task["goal"])
    radius = spec["radius"]
    if not isinstance(radius, int) or radius < 25:
        raise ValueError("Visibility radius must be at least 25 cells for car16")
    map_file = DATA / (task["map"] + ".map")
    width, actual = read_grid(map_file)
    known = actual.copy()
    controls = primitives()
    route = deque()
    moved = requests = updates = 0
    first_ms = replan_ms = navigation_ms = 0.0
    write_record(stream, {"kind": "episode", **spec})
    factory = engine_factory or Engine
    with factory(map_file, CONTROLS, spec["graph"], spec["algorithm"], goal, deadline) as engine:
        init_ms = engine.receive()["init_ms"]
        while True:
            tick = time.perf_counter()
            if deadline is not None and time.monotonic() >= deadline:
                raise TimeoutError("Series time budget expired")
            if moved % PERIOD == 0:
                number = moved // PERIOD
                protected = {p[1]*width+p[0] for p in (pose, goal)}
                saved = None if recorded is None else recorded[number]
                if saved is not None and (saved["moved"] != moved or saved["pose"] != list(pose)):
                    raise ValueError("World event at an unexpected robot position")
                block, clear = world_change(actual, protected, spec["seed"], number, spec["percent"], saved)
                updates = number
                navigation_ms += (time.perf_counter()-tick)*1000
                write_record(stream, {"kind": "world", "number": number, "moved": moved,
                                      "pose": pose, "block": block, "clear": clear})
                tick = time.perf_counter()
            block, clear = observe(actual, known, width, pose, radius)
            if requests == 0 or block or clear:
                navigation_ms += (time.perf_counter()-tick)*1000
                write_record(stream, {"kind": "request", "event": requests, "moved": moved,
                                      "pose": pose, "block": block, "clear": clear})
                answer = engine.query(pose, block, clear)
                if requests:
                    replan_ms += answer["plan_ms"]
                else:
                    first_ms = answer["plan_ms"]
                write_record(stream, {"kind": "result", "event": requests, **answer})
                requests += 1
                if answer["outcome"] == "no_path":
                    status = "no_path"
                    break
                tick = time.perf_counter()
                route = deque(answer["route"])
            if pose == goal:
                status = "reached"
                break
            if not route:
                raise RuntimeError("Route ended before reaching the goal")
            ident = route.popleft()
            dx, dy, heading, end, _, trace = controls[ident]
            x, y, h = pose
            if h != heading:
                raise RuntimeError("Invalid primitive heading")
            # All car16 traces fit in the smallest supported radius. This check
            # verifies execution against ground truth, independently of the planner.
            for a, b in trace:
                if not (0 <= x+a < width and 0 <= y+b < len(actual)//width) or actual[(y+b)*width+x+a]:
                    raise RuntimeError("Primitive intersects an obstacle on the actual map")
            moved += 1
            pose = (x+dx, y+dy, end)
            if on_move:
                on_move(ident, pose)
            navigation_ms += (time.perf_counter()-tick)*1000
            if pose == goal:
                status = "reached"
                break
    summary = dict(status=status, moves=moved, events=requests-1, world_updates=updates,
                   first_ms=first_ms, replan_ms=replan_ms, init_ms=init_ms, navigation_ms=navigation_ms,
                   total_ms=init_ms+first_ms+replan_ms+navigation_ms)
    write_record(stream, {"kind": "summary", **summary})
    return summary
