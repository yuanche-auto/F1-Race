"""Control expansion phase H – the ATV-QP learning MPC (docs/CX_H_protocol.md, revision 0 with its clarifications m1–m3,
n1, n2): the sets and the controllers; phase E's iteration loop (``cx_e.learn``) with the QP solver.

* Sets (section 2): ``HV`` – phase A's CXV cars (tuning); ``HT`` – ``sample_plants(nominal, 50, seed 382)[1:]`` (test);
  ``DEV`` – phase C's CXDEV cars (development; phase E's development cars); ``NOM``; secondary ``HT_LAG30`` (the 30 ms
  steering lag), ``HT_WET85`` (mu x 0.85). Case records as phase D's (``cx_d._case``).
* Kinds (section 3): ``atv`` (``cx_e.learn(..., "lmpc", solver="atv")`` – the error regression on), ``atv_nominal``
  (Gamma = 0), ``lmpc`` (phase E's NLP, unchanged – the formulation ablation), ``nmpc`` and ``identified`` (one scored
  lap, phase D's code path; ``identified``'s cache in results/CX/H/identified_cache).
"""
from __future__ import annotations

from pathlib import Path

from ..dynamics import DynVehicle
from . import cx_a, cx_d, cx_e

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "results" / "CX" / "H"
HT_SEED = 382
K_GRID = cx_e.K_GRID                              # {10, 20, 40} (section 4, clarification n2)
DEV_SETTING = {"K": 20}                           # section 9.3 (the gate: these laps only, clarification m1)
LEARN = {"atv": ("lmpc", "atv"), "atv_nominal": ("lmpc_nominal", "atv"), "lmpc": ("lmpc", "nlp")}
FIXED_KINDS = ("nmpc", "identified")
PRIMARY_KINDS = ("nmpc", "identified", "atv", "atv_nominal")
KINDS = PRIMARY_KINDS + ("lmpc",)
SECONDARY = ("HT_LAG30", "HT_WET85")


def setting_key(setting: dict) -> str:
    return cx_a.setting_key(setting)


def cases(name: str) -> list[dict]:
    from .l2 import sample_plants
    from .l8 import wet

    if name in ("NOM", "DEV"):
        return cx_e.cases(name)
    src = {"HV": (301, 10), "HT": (HT_SEED, 50), "HT_LAG30": (HT_SEED, 50), "HT_WET85": (HT_SEED, 50)}
    if name not in src:
        raise KeyError(name)
    seed, n = src[name]
    out = []
    for i, (info, plant) in enumerate(sample_plants(DynVehicle(), n, seed)[1:], start=1):
        kw: dict = {}
        if name == "HT_LAG30":
            kw["tau"] = cx_a.LAG_TAU
        elif name == "HT_WET85":
            kw["wet"], plant = 0.85, wet(plant, 0.85)
        out.append(cx_d._case(name, f"{name}_car{i}", i, info, plant, f"seed{seed}_car{i}", **kw))
    return out


def case_by_name(name: str) -> dict:
    return next(c for c in cases(name.rsplit("_", 1)[0]) if c["name"] == name)


def learn(case: dict, kind: str, setting: dict) -> dict:
    """The initialisation laps and the ten learning laps of one car (phase E's loop, section 2), with the QP for ``atv``
    / ``atv_nominal`` and phase E's NLP for ``lmpc``."""
    e_kind, solver = LEARN[kind]
    r = cx_e.learn(case, e_kind, setting, solver=solver)
    r["kind"], r["solver"] = kind, solver
    return r


def fixed_lap(case: dict, kind: str) -> dict:
    if kind not in FIXED_KINDS:
        raise ValueError(kind)
    cx_d.ID_CACHE = OUT / "identified_cache"           # this process only (one job per process)
    rec = cx_d.lap(case, kind, {})
    rec["setting"], rec["setting_key"] = {}, "default"
    return rec


def run(case: dict, kind: str, setting: dict) -> dict:
    return learn(case, kind, setting) if kind in LEARN else fixed_lap(case, kind)


def select(records: list[dict]) -> dict:
    """Section 4: phase E's rule for ``atv`` (most completed tenth laps; ties by the mean tenth lap on the common cars;
    then K = 20, then the smaller K)."""
    return cx_e.select(records, kind="atv")


__all__ = ["cases", "case_by_name", "learn", "fixed_lap", "run", "select", "KINDS", "PRIMARY_KINDS", "LEARN", "K_GRID",
           "DEV_SETTING", "SECONDARY", "setting_key"]
