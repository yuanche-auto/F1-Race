"""L4: identification of the car from driving data."""
import pickle
from pathlib import Path

import numpy as np
import pytest

from f1race.dynamics import DynVehicle
from f1race.estimation.identify import PARAMS, data_from_run, identify, theta_of, vehicle_from

CACHE = Path(__file__).resolve().parents[1] / "results" / "L1" / "l1_plans.pkl"


def test_vehicle_from_roundtrip():
    nom = DynVehicle()
    info = {"mu": 0.95, "m": 1.02, "ClA": 1.1, "CdA": 0.9, "Iz": 1.05, "B": 0.9, "aero_front": 0.02}
    v = vehicle_from(theta_of(info), nom)
    assert np.isclose(v.mu_x, 0.95 * nom.mu_x) and np.isclose(v.mu_y, 0.95 * nom.mu_y)
    assert np.isclose(v.aero_front, nom.aero_front + 0.02) and np.isclose(v.B, 0.9 * nom.B)
    assert vehicle_from(theta_of({"nominal": True}), nom) == nom


def test_identification_recovers_parameters(track):
    """20 s of driving (the first corners) with a perturbed car identify all seven
    parameters (noise-free data)."""
    if not CACHE.exists():
        pytest.skip("run scripts/run_l1.py first")
    from f1race.control.dyn_tracking import DynTracker
    from f1race.experiments.l1 import dyn_reference
    from f1race.experiments.l2 import x0_from_plan
    from f1race.sim.dynamic import DynamicSim

    nom = DynVehicle()
    o = pickle.load(open(CACHE, "rb"))["plans"][0.93]
    info = {"mu": 1.02, "m": 1.03, "ClA": 0.94, "CdA": 1.05, "Iz": 1.1, "B": 0.92, "aero_front": -0.02}
    plant = vehicle_from(theta_of(info), nom)
    path, ff = dyn_reference(track, nom, o)
    ctl = DynTracker(path, ff, nom, 0.01, k_p=0.06, x_la=12.0, k_r=0.05, kv=8.0)
    r = DynamicSim(track, plant, 0.01).run(ctl, x0_from_plan(o, path), t_max=20.0)
    res = identify(data_from_run(r, 0.01), nom)
    err = np.abs(res["theta"] - theta_of(info))
    assert np.all(err < 0.01), dict(zip(PARAMS, err.round(4)))
    assert res["rms"] < 0.05
