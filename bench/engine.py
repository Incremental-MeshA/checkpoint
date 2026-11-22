import json
import os
import queue
import subprocess
import threading
import time
from pathlib import Path

from .data import ROOT


def executable():
    override = os.environ.get("MESH_PLANNER")
    if override:
        return Path(override).resolve(strict=True)
    name = "planner.exe" if os.name == "nt" else "planner"
    for directory in (ROOT / "build" / "Release", ROOT / "build"):
        if (directory / name).is_file():
            return directory / name
    raise FileNotFoundError("Build the CMake target planner first")


class Engine:
    """One native process per episode, retaining planner state across requests."""
    def __init__(self, map_file, primitives, graph, algorithm, goal, deadline=None, *, trace=False, state_steps=False):
        self.deadline = deadline
        self.lines = queue.Queue()
        self.process = subprocess.Popen(
            [str(executable()), str(map_file), str(primitives), graph, algorithm, *map(str, goal),
             *(["--trace-states"] if state_steps else ["--trace"] if trace else [])],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        def reader():
            for line in self.process.stdout:
                self.lines.put(line)
            self.lines.put(None)
        self.reader = threading.Thread(target=reader, daemon=True)
        self.reader.start()

    def receive(self):
        timeout = None if self.deadline is None else max(0, self.deadline - time.monotonic())
        try:
            line = self.lines.get(timeout=timeout)
        except queue.Empty:
            raise TimeoutError("Series time budget expired") from None
        if line is None:
            raise RuntimeError(f"Planner stopped with exit code {self.process.wait()}")
        return json.loads(line)

    def query(self, pose, block=(), clear=()):
        values = (*pose, len(block), len(clear), *block, *clear)
        self.process.stdin.write(" ".join(map(str, values)) + "\n")
        self.process.stdin.flush()
        return self.receive()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        if exc_type:
            self.process.kill()
        self.process.stdin.close()
        self.process.wait()
        self.reader.join()
        self.process.stdout.close()
