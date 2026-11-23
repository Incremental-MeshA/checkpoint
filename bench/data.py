"""Download only the selected official assets and verify the offline corpus."""
import hashlib
import io
import json
import os
import tempfile
import urllib.request
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data"
LOCK = ROOT / "config" / "sources.json"
REVISION = "12574ff03d8d61ac153b4e4224eac2229a9577e1"
RAW = f"https://raw.githubusercontent.com/PathPlanning/MeshAStar/{REVISION}"
MAPS = ["Labyrinth", "Moscow_0_512", "32room_000"]


def digest(data):
    return hashlib.sha256(data).hexdigest()


def assets():
    yield "base_control_set.txt", f"{RAW}/data/base_control_set.txt", None
    yield "MeshAStar-LICENSE.txt", f"{RAW}/LICENSE.txt", None
    yield "base_mesh_info.txt", f"{RAW}/data/base_mesh_info.txt", None
    for name in MAPS:
        if name == "32room_000":
            url = f"https://www.movingai.com/benchmarks/room/{name}"
            yield f"{name}.map", url + ".map.zip", f"{name}.map"
            yield f"{name}.map.scen", url + ".map-scen.zip", f"{name}.map.scen"
        else:
            for suffix in (".map", ".map.scen"):
                yield name + suffix, f"{RAW}/maps/{name}{suffix}", None


def prepare():
    DATA.mkdir(parents=True, exist_ok=True)
    lock = json.loads(LOCK.read_text())
    results = {}
    for name, url, member in assets():
        path = DATA / name
        if path.exists():
            content = path.read_bytes()
        else:
            with urllib.request.urlopen(url, timeout=60) as response:
                content = response.read()
            if member:
                with zipfile.ZipFile(io.BytesIO(content)) as archive:
                    content = archive.read(member)
        checksum = digest(content)
        if checksum != lock[name]["sha256"] or url != lock[name]["url"]:
            raise ValueError(f"SHA-256 mismatch for {name}")
        results[name] = {"url": url, "sha256": checksum}
        if not path.exists():
            with tempfile.NamedTemporaryFile(dir=DATA, delete=False) as staging:
                staging.write(content)
            try:
                os.replace(staging.name, path)
            finally:
                Path(staging.name).unlink(missing_ok=True)
    return results


if __name__ == "__main__":
    prepare()


