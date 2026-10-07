import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))


@pytest.fixture(scope="session")
def track():
    from fetch_tracks import fetch

    from f1race.track import Track

    fetch("Spielberg")
    return Track.load("Spielberg")


@pytest.fixture(scope="session")
def vehicle():
    from f1race.vehicle import Vehicle

    return Vehicle()


@pytest.fixture(scope="session")
def mincurv(track, vehicle):
    from f1race.planning.mincurv import min_curvature_line

    return min_curvature_line(track, vehicle, margin=0.3)


@pytest.fixture
def circle_path():
    """Closed circle of radius 60 m sampled every ~1 m."""
    from f1race.path import Path as RPath

    R = 60.0
    th = np.linspace(0.0, 2.0 * np.pi, 377, endpoint=False)
    return R, RPath.from_points(np.column_stack([R * np.cos(th), R * np.sin(th)]))
