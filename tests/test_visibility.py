import io
import json
import unittest
import tempfile
from pathlib import Path
from contextlib import redirect_stdout
from unittest.mock import patch

from bench.__main__ import replay
from bench.navigation import primitives
from bench.select import MANIFEST
from bench.visibility import navigate_visible, observe, world_change


class Visibility(unittest.TestCase):
    def test_disk_boundary_and_memory(self):
        for radius in (25, 50, 100, 200):
            centre = radius + 1
            width = 2 * centre + 1
            pose = (centre, centre, 0)
            actual = bytearray(width*width)
            known = actual.copy()
            near = centre*width + centre+radius
            outside = near + width
            actual[near] = actual[outside] = 1
            self.assertEqual(observe(actual, known, width, pose, radius), ([near], []))
            actual[near] = 0
            self.assertEqual(observe(actual, known, width, (0,0,0), radius), ([], []))
            self.assertEqual(known[near], 1)
            self.assertEqual(observe(actual, known, width, pose, radius), ([], [near]))
            self.assertEqual(known[outside], 0)
        actual = bytearray(12)
        actual[3] = 1
        self.assertEqual(observe(actual, bytearray(12), 4, (0,1,0), 1), ([], []))

    def test_world_percentages_and_seed(self):
        grid = bytearray([0]*800 + [1]*200)
        protected = {0, 1}
        snapshot = grid.copy()
        block, clear = world_change(grid, protected, 47, 0, 0.5)
        self.assertEqual(len(block), 40)
        self.assertFalse(clear)
        self.assertEqual((block, clear), world_change(snapshot, protected, 47, 0, 0.5))
        before = grid.copy()
        block, clear = world_change(grid, protected, 47, 1, 0.5)
        self.assertEqual((len(block), len(clear)), (4, 4))
        self.assertTrue(all(not before[c] for c in block))
        self.assertTrue(all(before[c] for c in clear))
        self.assertFalse(protected.intersection(block + clear))
        self.assertEqual(sum(before), sum(grid))

    def test_observe_each_primitive_and_delayed_edits(self):
        width = 110
        goal = (60,1,0)
        spec = dict(task=dict(map="fixture", start=(0,1,0), goal=goal), graph="mesh",
                    algorithm="astar", seed=2, percent=0.5, radius=25, scenario="limited_visibility")
        events = [
            dict(moved=0, pose=[0,1,0], block=list(range(26,43)), clear=[]),
            dict(moved=20, pose=[20,1,0], block=[82,83], clear=[26,27]),
            dict(moved=40, pose=[40,1,0], block=[0,1], clear=[28,29]),
        ]
        calls = []
        class StraightLine:
            def __init__(self, *args):
                pass
            def __enter__(self):
                return self
            def __exit__(self, *args):
                pass
            def receive(self):
                return {"init_ms": 0}
            def query(self, pose, block, clear):
                calls.append((pose[0], block, clear))
                return {"plan_ms": 0, "outcome": "found", "route": [1]*(goal[0]-pose[0])}

        stream = io.StringIO()
        controls = {1: (1, 0, 0, 0, 1, [(0,0), (1,0)])}
        with patch.multiple("bench.visibility", Engine=StraightLine, primitives=lambda: controls,
                            read_grid=lambda _: (width, bytearray(width*3))):
            summary = navigate_visible(spec, stream, recorded=events)
        self.assertEqual((summary["status"], summary["moves"], summary["world_updates"]), ("reached",60,2))
        self.assertEqual(calls[0], (0, [], []))
        self.assertIn((2,[26],[]), calls)
        self.assertIn((20,[],[26,27]), calls)
        self.assertIn((40,[],[28,29]), calls)
        self.assertIn((58,[82],[]), calls)
        self.assertIn((59,[83],[]), calls)
        self.assertFalse(any(0 in block or 1 in block for _,block,_ in calls))
        self.assertEqual(summary["events"], len(calls)-1)
        logged = [json.loads(row) for row in stream.getvalue().splitlines()]
        self.assertEqual([row["moved"] for row in logged if row["kind"]=="world"], [0,20,40])


    def test_real_episode_can_replay_world_or_regenerate_it(self):
        task = next(t for t in json.loads(MANIFEST.read_text())["tasks"]
                    if t["id"] == "32room_000-L188")
        from bench.navigation import seed_for
        spec = dict(task=task, algorithm="astar", graph="lattice", percent=0.5, radius=25,
                    seed=seed_for(20261122, task["id"], 0), scenario="limited_visibility")
        with tempfile.TemporaryDirectory() as temp:
            log = Path(temp)/"episode.jsonl"
            with log.open("w") as stream:
                summary = navigate_visible(spec, stream)
            self.assertGreaterEqual(summary["world_updates"], 1)
            with redirect_stdout(io.StringIO()):
                with patch("bench.visibility.random.Random", side_effect=AssertionError("unexpected RNG")):
                    replay(log)
                replay(log, regenerate=True)
        self.assertTrue(all(a*a+b*b <= 25*25 for *_, trace in primitives().values() for a,b in trace))
