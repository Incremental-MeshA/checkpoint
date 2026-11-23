import io
import json
import unittest
from collections import Counter
from unittest.mock import patch

from bench.__main__ import clean
from bench.data import DATA, LOCK, MAPS, ROOT, digest
from bench.navigation import navigate, primitives, read_grid, seed_for
from bench.select import MANIFEST


class Navigation(unittest.TestCase):
    def test_pinned_snapshot_matches_original_hashes(self):
        archive = ROOT / "results" / "full-pinned"
        for name, expected in json.loads((archive / "inputs.sha256.json").read_text()).items():
            with self.subTest(file=name):
                self.assertEqual(digest((archive / "inputs" / name).read_bytes()), expected)

    def test_published_inputs(self):
        for name, source in json.loads(LOCK.read_text()).items():
            self.assertEqual(digest((DATA/name).read_bytes()), source["sha256"])
        self.assertEqual(Counter(p[2] for p in primitives().values()), dict.fromkeys(range(16), 24))
        tasks = json.loads(MANIFEST.read_text())["tasks"]
        quick = [t for t in tasks if t["quick"]]
        self.assertEqual(Counter(t["map"] for t in quick), dict.fromkeys(MAPS, 3))
        for task in tasks:
            row = (DATA/(task["map"]+".map.scen")).read_text().splitlines()[task["line"]-1].split()
            self.assertEqual(list(map(int, row[4:8])), task["start"][:2]+task["goal"][:2])
            self.assertGreater(task["initial_moves"], 10)

    def test_recorded_events_and_seed_replay(self):
        task = json.loads(MANIFEST.read_text())["tasks"][0]
        spec = dict(task=task, algorithm="mpgaa", graph="mesh", percent=10.0,
                    seed=seed_for(20261122, task["id"], 0), scenario="navigation_replanning")

        def play(requests=None):
            stream = io.StringIO()
            navigate(spec, stream, recorded=requests)
            return [json.loads(line) for line in stream.getvalue().splitlines()]

        original = play()
        requests = [r for r in original if r["kind"] == "request"]
        answers = [r for r in original if r["kind"] == "result"]
        self.assertGreaterEqual(len(requests), 2)
        self.assertEqual(answers[0]["cost"], task["initial_cost"])
        width, grid = read_grid(DATA/(task["map"]+".map"))
        obstacle_count = sum(grid)
        for event, request in enumerate(requests):
            self.assertEqual(request["moved"], 10*event)
            block, clear = request["block"], request["clear"]
            self.assertEqual(len(block), round(len(grid)*spec["percent"]/200) if event else 0)
            self.assertEqual(len(set(clear)), len(block))
            for c in block:
                self.assertEqual(grid[c], 0)
                grid[c] = 1
            for c in clear:
                self.assertEqual(grid[c], 1)
                grid[c] = 0
            self.assertEqual(sum(grid), obstacle_count)
            for x, y, _ in (request["pose"], task["goal"]):
                self.assertEqual(grid[y*width+x], 0)
        self.assertEqual(clean(original), clean(play()))
        with patch("bench.navigation.changes", side_effect=AssertionError("RNG used during replay")):
            self.assertEqual(clean(original), clean(play(requests)))
