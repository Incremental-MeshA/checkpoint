"""Loopback viewer for one episode or a side-by-side comparison."""
import json
import math
import threading
import webbrowser
from contextlib import contextmanager
from functools import lru_cache
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

from .data import DATA, MAPS, ROOT
from .engine import Engine
from .navigation import CONTROLS, primitives, read_grid, write_record
from .select import MANIFEST
from .__main__ import VARIANTS, GRAPHS, SCENARIOS

ASSETS = Path(__file__).with_name("viewer")


@lru_cache(maxsize=1)
def catalog():
    scenarios = {}
    for filename in ("benchmark.json", "visibility.json"):
        cfg = json.loads((ROOT / "config" / filename).read_text())
        scenarios[cfg["scenario"]] = {k: cfg[k] for k in
                                      ("seed", "change_percent", "radius", "radii") if k in cfg}
    return dict(maps=MAPS, tasks=json.loads(MANIFEST.read_text())["tasks"],
                algorithms=list(dict.fromkeys(a for a, _ in VARIANTS)), variants=VARIANTS, graphs=GRAPHS, scenarios=scenarios)


@lru_cache(maxsize=1)
def motion_library():
    controls = primitives()
    result = {}
    for section in CONTROLS.read_text().split("===== prim description: =====")[1:]:
        ident = int(section.split("prim ID is:")[1].splitlines()[0])
        dx, dy, start, end, _, trace = controls[ident]
        rows = section.split("trajectory is:")[1].split("---")[0].strip().splitlines()
        curve = [[round(float(v), 4) for v in row.split()] for row in rows]
        result[ident] = dict(dx=dx, dy=dy, start=start, end=end, trace=trace, curve=curve)
    return result


@lru_cache(maxsize=12)
def grid_for(name):
    if name not in MAPS:
        raise ValueError("Unknown map")
    return read_grid(DATA / (name + ".map"))


def episode_spec(form):
    name = form["map"]
    width, grid = grid_for(name)
    cfg = catalog()["scenarios"].get(form["scenario"])
    if cfg is None or (form["algorithm"], form["graph"]) not in VARIANTS:
        raise ValueError("Select a scenario, algorithm, and graph from the lists.")
    if form["task"] == "manual":
        task = dict(id="manual", map=name)
        for field in ("start", "goal"):
            pose = form[field]
            if not isinstance(pose, list) or len(pose) != 3 or any(type(v) is not int for v in pose):
                raise ValueError("A pose requires a cell and a heading.")
            x, y, heading = pose
            if not (0 <= x < width and 0 <= y < len(grid)//width and 0 <= heading < 16
                    and not grid[y*width+x]):
                raise ValueError("Select a free cell and a valid heading.")
            task[field] = pose
    else:
        task = next((t for t in catalog()["tasks"] if t["map"] == name and t["id"] == form["task"]), None)
        if task is None:
            raise ValueError("Select a task for this map.")
    percent = float(form["percent"])
    if not math.isfinite(percent) or not 0 <= percent <= 100:
        raise ValueError("The percentage of changed cells must be between 0 and 100.")
    seed = int(str(form["seed"]))
    spec = dict(task=task, seed=seed, percent=percent, scenario=form["scenario"],
                algorithm=form["algorithm"], graph=form["graph"])
    if "radii" in cfg:
        if type(form["radius"]) is not int or form["radius"] not in cfg["radii"]:
            raise ValueError("Select a sensor radius from the list.")
        spec["radius"] = form["radius"]
    return spec


class Episode:
    def __init__(self, key, output, state_steps=False):
        self.state_steps = state_steps
        self.key = key
        self.output = output
        self.lock = threading.Lock()
        self.cancelled = False
        self.engine = None

    def cancel(self):
        with self.lock:
            self.cancelled = True
            if self.engine and self.engine.process.poll() is None:
                self.engine.process.kill()

    @contextmanager
    def open_engine(self, *args):
        with self.lock:
            if self.cancelled:
                raise InterruptedError()
            self.engine = Engine(*args, trace=True, state_steps=self.state_steps)
        try:
            with self.engine as engine:
                yield engine
        finally:
            with self.lock:
                self.engine = None

    def write(self, line):
        if self.cancelled:
            raise InterruptedError()
        self.output.write(line.encode("utf-8"))

    def flush(self):
        self.output.flush()

    def move(self, ident, pose):
        write_record(self, dict(kind="move", primitive=ident, pose=pose))


class ViewerServer(ThreadingHTTPServer):
    def __init__(self, port):
        super().__init__(("127.0.0.1", port), Handler)
        self.lock = threading.Lock()
        self.active = {}

    def replace(self, slot, episode):
        with self.lock:
            previous = self.active.pop(slot, None)
            if previous:
                previous.cancel()
            if episode:
                self.active[slot] = episode


class Handler(BaseHTTPRequestHandler):
    def reply(self, data, mime, status=200):
        self.send_response(status)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def json(self, data, status=200):
        self.reply(json.dumps(data, ensure_ascii=False, separators=(",", ":")).encode(),
                   "application/json; charset=utf-8", status)

    def do_GET(self):
        path = urlsplit(self.path).path
        assets = {"/": ("index.html", "text/html; charset=utf-8"),
                  "/viewer.js": ("viewer.js", "text/javascript; charset=utf-8"),
                  "/episode.js": ("episode.js", "text/javascript; charset=utf-8"),
                  "/map.js": ("map.js", "text/javascript; charset=utf-8"),
                  "/viewer.css": ("viewer.css", "text/css; charset=utf-8")}
        if path in assets:
            name, mime = assets[path]
            self.reply((ASSETS / name).read_bytes(), mime)
        elif path == "/api/catalog":
            self.json(catalog())
        elif path == "/api/primitives":
            self.json(motion_library())
        elif path.startswith("/api/maps/") and path[10:] in MAPS:
            width, grid = grid_for(path[10:])
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("X-Map-Width", str(width))
            self.send_header("Content-Length", str(len(grid)))
            self.end_headers()
            self.wfile.write(grid)
        else:
            self.send_error(404)

    def do_POST(self):
        origin = "http://127.0.0.1:" + str(self.server.server_port)
        if (self.headers.get("Host") != origin[7:] or self.headers.get("Origin", origin) != origin
                or self.headers.get_content_type() != "application/json"):
            return self.send_error(403)
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 8192:
                raise ValueError("Invalid request size")
            form = json.loads(self.rfile.read(length))
            slot = form.get("slot", "left")
            if slot not in ("left", "right"):
                raise ValueError("Unknown panel")
            key = form["key"]
            if not isinstance(key, str) or not key:
                raise ValueError("Missing episode ID")
            if self.path == "/api/cancel":
                with self.server.lock:
                    current = self.server.active.get(slot)
                    if current and current.key == key:
                        current.cancel()
                return self.json({})
            if self.path != "/api/run":
                return self.send_error(404)
            spec = episode_spec(form)
        except (KeyError, ValueError, TypeError, OverflowError) as err:
            return self.json({"error": str(err)}, 400)
        job = Episode(key, self.wfile, state_steps=form.get("state_steps") is True)
        self.server.replace(slot, job)
        try:
            self.send_response(200)
            self.send_header("Content-Type", "application/x-ndjson; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            SCENARIOS[spec["scenario"]](spec, job, engine_factory=job.open_engine, on_move=job.move)
        except (ConnectionError, InterruptedError):
            pass
        except (ValueError, RuntimeError, OSError) as err:
            if not job.cancelled:
                try:
                    write_record(job, {"kind": "error", "error": str(err)})
                except OSError:
                    pass
        finally:
            with self.server.lock:
                if self.server.active.get(slot) is job:
                    del self.server.active[slot]


def serve(port=0, open_browser=True):
    with ViewerServer(port) as server:
        url = f"http://127.0.0.1:{server.server_port}/"
        print(url, flush=True)
        if open_browser:
            webbrowser.open(url)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            for slot in ("left", "right"):
                server.replace(slot, None)
