import heapq
import itertools
import json
import random
import tempfile
import unittest
from pathlib import Path

from bench.data import DATA
from bench.engine import Engine
from bench.navigation import CONTROLS, changes, primitives, read_grid

VARIANTS = list(itertools.product(("astar", "dstar_lite", "mpgaa"), ("lattice", "mesh")))
UNIT = 1 << 52


def dijkstra(width, grid, start, goal, controls):
    """Independent state lattice with exact binary64-length sums as Python integers."""
    best = {start: 0}
    queue = [(0, start)]
    by_heading = [[] for _ in range(16)]
    for p in controls.values():
        by_heading[p[2]].append(p)
    while queue:
        cost, pose = heapq.heappop(queue)
        if best[pose] != cost:
            continue
        if pose == goal:
            return cost / UNIT
        x, y, heading = pose
        for dx, dy, _, end, length, trace in by_heading[heading]:
            if any(not (0 <= x+a < width and 0 <= y+b < len(grid)//width)
                   or grid[(y+b)*width+x+a] for a, b in trace):
                continue
            successor = (x+dx, y+dy, end)
            new = cost + int(length * UNIT)
            if new < best.get(successor, float("inf")):
                best[successor] = new
                heapq.heappush(queue, (new, successor))
    return None


def grid_file(path, width, height):
    path.write_text(f"type octile\nheight {height}\nwidth {width}\nmap\n"+
                    ("."*width+"\n")*height)


def primitive(ident, start, end, cells):
    x, y = cells[-1]
    return (f"===== prim description: =====\nstart heading (number): {start}\n"
            f"goal state (i, j, heading num): {y} {x} {end}\n"
            f"length is: {len(cells)-1}\nprim ID is: {ident}\ncollision is:\n" +
            "".join(f"{b} {a}\n" for a,b in cells)+"---\nprim end\n")


class Planners(unittest.TestCase):
    def test_shared_mesh_edge_reports_both_complete_motions(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "branch.map"
            controls = Path(tmp) / "branch.txt"
            grid_file(path, 6, 4)
            controls.write_text(primitive(13,0,0,[(0,0),(1,0),(2,0)]) +
                                primitive(71,0,1,[(0,0),(1,0),(1,1)]))
            for algorithm in ("astar", "dstar_lite"):
                with Engine(path, controls, "mesh", algorithm, (3,1,0), trace=True) as engine:
                    engine.receive()
                    result = engine.query((1,1,0))
                    shared = [edge for _, edges in result["edge_samples"] for edge in edges
                              if edge[:5] == [1,1,2,1,-1]]
                    self.assertTrue(shared, algorithm)
                    self.assertEqual(shared[0][8:], [[1,1], [13,71]])

    def test_checked_cells_follow_collision_trace_and_reset_per_query(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "line.map"
            controls = Path(tmp) / "straight.txt"
            grid_file(path, 4, 1)
            controls.write_text(primitive(1, 0, 0, [(0,0), (1,0), (2,0), (3,0)]))
            start, goal = (0,0,0), (3,0,0)
            for graph, reads in [("lattice", 13), ("mesh", 17)]:
                with self.subTest(graph=graph), Engine(path, controls, graph, "astar", goal, trace=True) as e:
                    e.receive()
                    first = e.query(start)
                    candidate = first["edge_samples"][0][1][0]
                    self.assertEqual(candidate[:6], [0,0,3,0,1,1] if graph == "lattice" else [0,0,1,0,-1,1])
                    if graph == "mesh":
                        for _, edges in first["edge_samples"]:
                            for edge in edges:
                                self.assertEqual(edge[8:], [[0,0], [1]])
                    self.assertEqual((first["checked_cells"], first["cell_checks"]), (4, reads))
                    self.assertEqual(first["cell_frames"][-1], [len(first["trace"]), 4, reads])
                    blocked = e.query(start, [1])
                    self.assertEqual(blocked["outcome"], "no_path")
                    self.assertEqual((blocked["checked_cells"], blocked["cell_checks"]), (2, 4))
                    self.assertEqual(blocked["trace"], [])
                    same = e.query(goal)
                    self.assertEqual((same["checked_cells"], same["cell_checks"]), (0, 0))

    def check_sequence(self, map_path, controls_path, goal, queries):
        controls = primitives(controls_path)
        width, initial = read_grid(map_path)
        grid = initial.copy()
        expected = []
        for pose, block, clear in queries:
            for c in block:
                grid[c] = 1
            for c in clear:
                grid[c] = 0
            expected.append(dijkstra(width, grid, pose, goal, controls))
        for algo, graph in VARIANTS:
            with self.subTest(algo=algo, graph=graph):
                grid = initial.copy()
                with Engine(map_path, controls_path, graph, algo, goal) as engine:
                    engine.receive()
                    for (pose, block, clear), optimal in zip(queries, expected):
                        for c in block:
                            grid[c] = 1
                        for c in clear:
                            grid[c] = 0
                        result = engine.query(pose, block, clear)
                        phases = [result[k] for k in ("apply_ms", "changed_edges_ms", "search_ms", "path_ms")]
                        self.assertTrue(all(t >= 0 for t in phases))
                        self.assertAlmostEqual(sum(phases), result["plan_ms"], places=8)
                        self.assertEqual(result["cost"], optimal)
                        self.assertEqual(result["outcome"], "no_path" if optimal is None else "found")
                        ticks = 0
                        for ident in result["route"]:
                            dx, dy, h, end, length, trace = controls[ident]
                            self.assertEqual(h, pose[2])
                            for a, b in trace:
                                x, y = pose[0]+a, pose[1]+b
                                self.assertTrue(0 <= x < width and 0 <= y < len(grid)//width)
                                self.assertEqual(grid[y*width+x], 0)
                            ticks += int(length*UNIT)
                            pose = (pose[0]+dx, pose[1]+dy, end)
                        if optimal is not None:
                            self.assertEqual(pose, goal)
                            self.assertEqual(ticks/UNIT, optimal)
                        self.assertLessEqual(result["expanded_cells"], result["expanded"])
        return expected

    def test_real_car16(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/"test.map"
            grid_file(path, 14, 10)
            pose = (1, 4, 0)
            goal = (11, 4, 0)
            queries = [(pose, [], []), (pose, [4*14+5], []),
                       ((2, 4, 0), [4*14+6], [4*14+5]), ((3, 4, 0), [], [4*14+6]),
                       ((3, 4, 1), [], []), (goal, [], [])]
            costs = self.check_sequence(path, CONTROLS, goal, queries)
            self.assertGreater(costs[1], costs[0])
            self.assertGreater(costs[2], costs[3])

    def test_incremental_reuse(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"straight.map"
            grid_file(path, 22, 6)
            start, goal = (1, 2, 0), (17, 2, 0)
            for algo, graph in VARIANTS:
                with self.subTest(algo=algo, graph=graph), Engine(path, CONTROLS, graph, algo, goal) as engine:
                    engine.receive()
                    first = engine.query(start)
                    second = engine.query(start)
                    self.assertEqual(first["cost"], second["cost"])
                    self.assertGreater(first["search_expanded"], 0)
                    if algo == "astar":
                        self.assertEqual(first["search_expanded"], second["search_expanded"])
                    else:
                        self.assertEqual(second["search_expanded"], 0)

    def test_shared_prefix_unreachable_and_recovery(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/"test.map"
            grid_file(path, 7, 3)
            raw = Path(temp)/"controls.txt"
            raw.write_text(primitive(1,0,0,[(0,0),(1,0),(2,0),(3,0)])+
                           primitive(2,0,1,[(0,0),(1,0),(2,0),(2,1)]))
            pose = (0,1,0)
            goal = (5,2,1)
            results = self.check_sequence(path,raw,goal,[(pose,[],[]),(pose,[8],[]),
                                                         (pose,[],[8]),((3,1,0),[],[])])
            self.assertEqual(results, [6, None, 6, 3])

    def test_only_relevant_transitions_seed_updates(self):
        # Two parallel primitives, same endpoints but different traces and costs.
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"parallel.map"
            controls = Path(tmp)/"controls.txt"
            width = 5
            grid_file(path, width, 8)
            straight = [(0,0), (1,0), (2,0), (3,0), (4,0)]
            detour = [(0,0), (0,1), (1,1), (2,1), (3,1), (4,1), (4,0)]
            controls.write_text(primitive(1,0,0,straight) + primitive(2,0,0,detour))
            start, goal = (0,1,0), (4,1,0)
            a, b, far = width+1, width+2, 7*width
            queries = [(start,[],[]), (start,[far],[]), (start,[],[far]),
                       (start,[a,b],[]), (start,[],[a]), (start,[a],[b]),
                       (start,[],[a]), (start,[a,b],[]), (start,[],[a,b])]
            costs = self.check_sequence(path, controls, goal, queries)
            self.assertEqual(costs, [4,4,4,6,6,6,4,6,4])
            for algo, graph in VARIANTS:
                with self.subTest(algo=algo, graph=graph), Engine(path, controls, graph, algo, goal) as engine:
                    engine.receive()
                    results = [engine.query(*q) for q in queries]
                    seeds = [r["batch_seeds"] for r in results]
                    self.assertEqual(seeds[:3], [0,0,0])
                    if algo == "astar":
                        self.assertFalse(any(seeds))
                        continue
                    for r in results[1:3]:
                        self.assertEqual((r["search_expanded"], r["repair_expanded"]), (0,0))
                    if algo == "mpgaa":
                        self.assertEqual(seeds[3], 0)  # blocking alone never lowers h
                    if graph == "lattice":
                        self.assertEqual(seeds[4:6], [0,0])  # still blocked after the entire batch
                        self.assertEqual(seeds[-1], 1)  # two opened cells, one source
                    self.assertGreater(seeds[-1], 0)

    def test_seeded_mixed_batches(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"small.map"
            grid_file(path, 12, 12)
            rng = random.Random(145)
            blocked = set(rng.sample(range(12,132), 12))
            pose, goal = (1,0,0), (10,11,4)
            queries = [(pose, sorted(blocked), [])]
            for _ in range(12):
                clear = rng.sample(sorted(blocked),3)
                block = rng.sample(sorted(set(range(12,132))-blocked),3)
                blocked.difference_update(clear)
                blocked.update(block)
                queries.append((pose, block, clear))
            self.check_sequence(path,CONTROLS,goal,queries)

    def test_dstar_replays_berlin_visibility_requests(self):
        # Observations from full-pinned/visibility-25, with costs checked by lattice A*.
        fixtures = Path(__file__).with_name("fixtures")
        for case in ("Berlin_2_512-L220-dstar_lite-mesh-r0", "Berlin_2_512-L275-dstar_lite-mesh-r2"):
            header, *queries = map(json.loads, (fixtures / (case + ".jsonl")).read_text().splitlines())
            with self.subTest(case=case), Engine(DATA / (header["map"] + ".map"), CONTROLS,
                                                  "mesh", "dstar_lite", header["goal"]) as engine:
                engine.receive()
                for event, query in enumerate(queries):
                    with self.subTest(event=event):
                        answer = engine.query(query["pose"], query["block"], query["clear"])
                        cost = query["cost"]
                        self.assertEqual(answer["outcome"], "no_path" if cost is None else "found")
                        self.assertEqual(answer["cost"], cost)

    def test_dstar_repairs_a_prefix_after_collinear_start_moves(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "turn.map"
            controls = Path(tmp) / "controls.txt"
            grid_file(path, 16, 10)
            direct = [(x, 0) for x in range(5)]
            detour = [(0, 0)] + [(x, 1) for x in range(5)] + [(4, 0)]
            controls.write_text(primitive(1, 0, 0, direct) + primitive(2, 0, 0, detour))

            start, goal = (10, 7, 0), (14, 7, 0)
            blocked = 7 * 16 + 13
            with Engine(path, controls, "mesh", "dstar_lite", goal) as engine:
                engine.receive()
                self.assertEqual(engine.query(start)["route"], [1])
                # Heading 1 has no outgoing motion. The blocked prefix stays in
                # OPEN while the start moves back in three collinear steps.
                for i in (3, 2, 1):
                    pose = (start[0] - 3*i, start[1] - 2*i, 1)
                    result = engine.query(pose, [blocked] if i == 3 else [])
                    self.assertEqual(result["outcome"], "no_path")
                result = engine.query(start)
                self.assertEqual((result["cost"], result["route"]), (6, [2]))
                self.assertEqual(engine.query(start, [], [blocked])["route"], [1])

    def test_dstar_retains_successor_order_on_equal_costs(self):
        # An equal-cost edge reopens after the other one has become the cached best.
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"ties.map"
            controls = Path(tmp)/"ties.txt"
            grid_file(path, 5, 3)
            lower = [(0,0), (0,1), (1,1), (2,1), (2,0)]
            upper = [(0,0), (0,-1), (1,-1), (2,-1), (2,0)]
            controls.write_text(primitive(9,0,0,lower) + primitive(1,0,0,upper))
            start, goal = (1,1,0), (3,1,0)
            edits = [([],[]), ([2],[]), ([12],[2]), ([],[12]), ([12,2],[]), ([],[2,12])]
            self.check_sequence(path, controls, goal, [(start,b,c) for b,c in edits])
            for graph in ("lattice", "mesh"):
                with self.subTest(graph=graph), Engine(path, controls, graph, "dstar_lite", goal) as engine:
                    engine.receive()
                    results = [engine.query(start, b, c) for b,c in edits]
                    self.assertEqual([r["route"] for r in results], [[9],[9],[1],[9],[],[9]])
                    if graph == "lattice":
                        self.assertEqual(results[1]["batch_seeds"], 0)

    def test_balanced_sampler(self):
        grid = bytearray(i % 10 == 0 for i in range(512*512))
        for pct, count in [(0.1,131),(1,1311),(10,13107)]:
            block, clear = changes(grid,pct,{0,1},123)
            self.assertEqual((len(set(block)),len(set(clear))),(count,count))
            self.assertFalse((set(block)|set(clear)) & {0,1})
            self.assertTrue(all(not grid[c] for c in block))
            self.assertTrue(all(grid[c] for c in clear))
            self.assertEqual((block,clear),changes(grid,pct,{0,1},123))


if __name__ == "__main__":
    unittest.main()

