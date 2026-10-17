"""Download track data from the TUM FTM racetrack-database (LGPL-3.0).

The CSV files are not redistributed in this repository. They are fetched from
a pinned upstream commit and verified against SHA-256 checksums.

    python scripts/fetch_tracks.py            # all tracks listed below
    python scripts/fetch_tracks.py Spielberg  # a single track
"""
from __future__ import annotations

import hashlib
import sys
import urllib.request
from pathlib import Path

UPSTREAM = "https://raw.githubusercontent.com/TUMFTM/racetrack-database"
COMMIT = "e59595d1f3573b30d1ded6a08984935b957688e0"

SHA256 = {
    "Spielberg": "277b2854c4323021b10b6b63ebfe6a6cc971f494c0728202e910d67d7d65dc8c",
    "Monza": "4b5993986e67950df1b89efa03a4df02127f07b7213985917f0bad27ad3d48b6",
    "Budapest": "3c25a55841af6de1e78a366593108835738d78e75cedfb7acd11c9b36da53d5a",
    "Silverstone": "27c51454e81780a75c353d3e6c9f73dbbcd1e82139782e95bd722ade19736e39",
    "Catalunya": "8d6dc9d50e98bdecd818cb880a4b12bb5cec25d699a6b774e0b593c0f8a9da5e",
    "Sakhir": "bc3c23da42f9474dbf5700341037bb45521acaa2ad1389bd0a48540fe179ee7a",       # L8 validation
    "Zandvoort": "f3f2a82c9f41bcee8d7161a6ad6c0b8e6d4b902dd772459918c9fe7f7451bc59",    # L8 test
    "SaoPaulo": "7383caa2b7af9af6c321aa5442cc5b47b6cfe1a56f70b56f3216ba1a3c015a04",     # L8 test
    "Norisring": "8857d3c362ad2923c1f93c8d257498f50459770b9021adcc7969b71085c31d9a",    # L8: plans failed, replaced
    "Nuerburgring": "9fea2a9daa25311bb251be416140b6c55cb2e53bb4f69c28f512fbfce25a3f87", # L8 test
    "Austin": "60d46180320177ba49bec61bad390872039222dbab225fe12781ca662becda55",       # L8 test (replaces Norisring, plans failed)
}

DATA_DIR = Path(__file__).resolve().parents[1] / "data" / "tracks"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fetch(name: str, force: bool = False) -> Path:
    if name not in SHA256:
        raise KeyError(f"unknown track {name!r}; known: {sorted(SHA256)}")
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    path = DATA_DIR / f"{name}.csv"
    if path.exists() and not force and sha256(path) == SHA256[name]:
        return path
    url = f"{UPSTREAM}/{COMMIT}/tracks/{name}.csv"
    with urllib.request.urlopen(url, timeout=60) as resp:
        data = resp.read()
    digest = hashlib.sha256(data).hexdigest()
    if digest != SHA256[name]:
        raise RuntimeError(f"checksum mismatch for {name}: {digest}")
    path.write_bytes(data)
    return path


def main(argv: list[str]) -> None:
    names = argv or sorted(SHA256)
    for name in names:
        print(f"{name}: {fetch(name)}")


if __name__ == "__main__":
    main(sys.argv[1:])
