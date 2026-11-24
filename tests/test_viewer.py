import io
import json
import unittest
from collections import Counter
from functools import partial

from bench.__main__ import VARIANTS, GRAPHS, SCENARIOS, clean
from bench.engine import Engine
from bench.navigation import seed_for
from bench.navigation import CONTROLS, primitives
from bench.data import DATA
from bench.viewer import catalog, episode_spec, grid_for


class Viewer(unittest.TestCase):
    def test_mpgaa_cached_suffix_boundary(self):
        task = catalog()["tasks"][0]
        for graph in GRAPHS:
            with self.subTest(graph=graph), Engine(DATA / (task["map"] + ".map"), CONTROLS,
                    graph, "mpgaa", task["goal"], trace=True) as engine:
                engine.receive()
                first = engine.query(task["start"])
                cached = engine.query(task["start"])
                self.assertIsNone(first["reuse_from_edge"])
                self.assertTrue(first["route"])
                self.assertEqual(cached["route"], first["route"])
                self.assertEqual(cached["reuse_from_edge"], 0)
                self.assertEqual(cached["trace"], [])

    def test_logical_steps_distinguish_learning_and_cache_hit(self):
        task = catalog()["tasks"][0]
        with Engine(DATA / (task["map"] + ".map"), CONTROLS, "lattice", "mpgaa",
                    task["goal"], logical=True) as engine:
            engine.receive()
            first = engine.query(task["start"])
            cached = engine.query(task["start"])
            learned = first["walkthrough"]["learned_poses"]
            self.assertEqual(len(learned), first["walkthrough"]["raised"][0])
            self.assertEqual(cached["walkthrough"]["learned_poses"], [])
            self.assertTrue(first["walkthrough"]["started"])
            self.assertGreater(first["walkthrough"]["raised"][0], 0)
            self.assertEqual(cached["route"], first["route"])
            self.assertEqual(cached["reuse_from_edge"], 0)
            self.assertEqual(cached["walkthrough"]["raised"][0], 0)

    def test_traced_episodes_match_native_runs(self):
        for scenario in ("navigation_replanning", "limited_visibility"):
            task = next(t for t in catalog()["tasks"] if t["map"] == "32room_000" and t["band"] == "short")
            task_id = task["id"]
            for algo, graph in VARIANTS:
                with self.subTest(scenario=scenario, algorithm=algo, graph=graph):
                    spec = dict(task=task, algorithm=algo, graph=graph, scenario=scenario,
                                seed=seed_for(20261122, task_id, 0), percent=0.5)
                    if scenario == "limited_visibility":
                        spec["radius"] = 25
                    plain, traced = io.StringIO(), io.StringIO()
                    simulate = SCENARIOS[scenario]
                    simulate(spec, plain)
                    moves = []
                    simulate(spec, traced, engine_factory=partial(Engine, trace=True),
                             on_move=lambda ident, pose: moves.append((ident, pose)))
                    expected = [json.loads(s) for s in plain.getvalue().splitlines()]
                    records = [json.loads(s) for s in traced.getvalue().splitlines()]
                    width, _ = grid_for(task["map"])
                    reused = False
                    for record in records:
                        if record["kind"] != "result":
                            continue
                        trace = record.pop("trace")
                        samples = record.pop("edge_samples")
                        self.assertLessEqual(len(samples), 32)
                        for event, edges in samples:
                            self.assertTrue(1 <= event <= len(trace))
                            self.assertTrue(edges)
                            if graph == "mesh":
                                for sx, sy, tx, ty, primitive, _, _, _, (ox, oy), ids in edges:
                                    self.assertTrue(ids)
                                    if primitive >= 0:
                                        self.assertEqual(ids, [primitive])
                                    for ident in ids:
                                        path = primitives()[ident][-1]
                                        self.assertIn(((sx-ox, sy-oy), (tx-ox, ty-oy)), zip(path, path[1:]))
                        frames = record.pop("cell_frames")
                        unique, reads = record.pop("checked_cells"), record.pop("cell_checks")
                        self.assertLessEqual(unique, reads)
                        previous = [0, 0, 0]
                        for end, cells, checks in frames:
                            self.assertGreater(end, previous[0])
                            self.assertGreaterEqual(cells, previous[1])
                            self.assertGreaterEqual(checks, previous[2])
                            previous = [end, cells, checks]
                        if trace:
                            self.assertEqual(previous, [len(trace), unique, reads])
                        counts = Counter(code % 4 for code in trace)
                        self.assertEqual(counts[0], record["search_expanded"])
                        self.assertEqual(counts[1], record["repair_expanded"])
                        if algo.startswith("dstar_lite"):
                            self.assertEqual(counts[2], record["batch_seeds"])
                            if record["event"] == 0:
                                x, y, _ = task["goal"]
                                self.assertEqual(trace[0] // 8, y * width + x)
                        boundary = record.pop("reuse_from_edge")
                        if boundary is not None:
                            self.assertTrue(algo.startswith("mpgaa") or algo == "md_scan")
                            self.assertGreaterEqual(boundary, 0)
                            reused = True
                    self.assertEqual(clean(records), clean(expected))
                    self.assertEqual(len(moves), records[-1]["moves"])
                    if records[-1]["status"] == "reached":
                        self.assertEqual(list(moves[-1][1]), task["goal"])
                    if algo.startswith("mpgaa"):
                        self.assertTrue(reused)

    def test_manual_pose_and_exact_seed(self):
        task = catalog()["tasks"][0]
        form = dict(map=task["map"], task="manual", start=task["start"], goal=task["goal"],
                    graph="mesh", algorithm="mpgaa", percent=.1, scenario="limited_visibility",
                    radius=25, seed="18446744073709551613")
        self.assertEqual(episode_spec(form)["seed"], 18446744073709551613)
        width, grid = grid_for(task["map"])
        cell = grid.index(1)
        for invalid in [dict(start=[cell % width, cell // width, 0]), dict(goal=[-1, 0, 0]),
                        dict(start=[1, 1, 16]), dict(radius=10), dict(percent="NaN")]:
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                episode_spec({**form, **invalid})
