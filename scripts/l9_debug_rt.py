"""L9 debugging aid (not a protocol run): the first seconds of one real-time lap of an IPOPT controller
on Spielberg's nominal car, step by step (plan alignment, segment executed, command, lateral offset).

    python scripts/l9_debug_rt.py nmpc_reserve 3.0
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import numpy as np  # noqa: E402

from f1race.dynamics import DynVehicle  # noqa: E402
from f1race.experiments.l1 import dyn_reference  # noqa: E402
from f1race.experiments.l2 import x0_from_plan  # noqa: E402
from f1race.experiments.l9 import DT, RealTimeMPCController, make_mpc, plans  # noqa: E402
from f1race.sim.dynamic import DynamicSim  # noqa: E402
from f1race.track import Track  # noqa: E402

kind, t_end = sys.argv[1], float(sys.argv[2])
impl = sys.argv[3] if len(sys.argv) > 3 else "ipopt"
track, nom = Track.load("Spielberg"), DynVehicle()
eta = {"nmpc": 0.95, "nmpc_reserve": 0.88}[kind]
o = plans()[eta]
path, _ = dyn_reference(track, nom, o)
mpc = make_mpc(impl, track, nom, o, eta, name="debug")
ctl = RealTimeMPCController(mpc, DT, "realtime")
orig = ctl.__call__
rows = []


def traced(t, x):
    u = orig(t, x)
    xf = ctl.frenet(x)
    act = ctl.active
    j = int(np.floor((t - act[0]) / 0.05 + 1e-9)) if act else -1
    rows.append((t, xf[0], xf[1], xf[2], x[3], x[6], act[0] if act else np.nan, j, u[0], u[1], u[2],
                 ctl.pending[0] if ctl.pending else np.nan))
    return u


ctl.__call__ = traced
sim = DynamicSim(track, nom, DT)


class C:                                         # sim.run calls controller(t, x)
    def __call__(self, t, x):
        return traced(t, x)


r = sim.run(C(), x0_from_plan(o, path), t_max=t_end)
print(f"{r.stop_reason} at {r.progress[-1]:.0f} m after {r.t[-1]:.2f} s")
print("   t      s      n     xi     vx   delta | plan_t0  j | drate    Fd     Fb | pending")
for row in rows[::5]:
    print("{:5.2f} {:6.1f} {:6.2f} {:6.3f} {:6.1f} {:7.4f} | {:6.2f} {:3d} | {:6.3f} {:6.0f} {:6.0f} | {:5.2f}".format(*row))
for c in ctl.calls[:12]:
    print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in c.items()})
