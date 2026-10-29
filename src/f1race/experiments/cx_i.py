"""Control expansion phase I – the learning MPC's follow-ups (docs/CX_I_protocol.md, revision 0 with its clarifications m1,
m2, n1, n2); candidate I-1, the speed bound per lap: the sets and the controllers; phase E's iteration loop
(``cx_e.learn``) with the bound.

* Sets (section 4): ``IV`` – phase A's CXV cars (tuning); ``IT`` – ``sample_plants(nominal, 50, seed 383)[1:]`` (test);
  ``DEV`` – phase C's CXDEV cars (development; phases E's and H's development cars); ``NOM``; secondary ``IT_LAG30`` (the
  30 ms steering lag), ``IT_WET85`` (mu x 0.85). Case records as phase D's (``cx_d._case``). The benchmark is registered
  only by a revision after the gate (clarification m1); until then only NOM and DEV are driven.
* Kinds (section 3): ``pb`` (phase E's NLP with the speed bound, the error regression on), ``pb_nominal`` (Gamma = 0),
  ``lmpc`` (phase E's NLP, no bound – the bound's ablation), ``nmpc`` and ``identified`` (one scored lap, phase D's code
  path; ``identified``'s cache in results/CX/I/identified_cache).
"""
from __future__ import annotations

from pathlib import Path

from ..dynamics import DynVehicle
from . import cx_a, cx_d, cx_e

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "results" / "CX" / "I"
IT_SEED = 383
K = 20                                            # fixed (section 3.1, clarification n1)
EPS_GRID = [{"K": K, "eps": e} for e in (0.02, 0.04, 0.08)]
DEV_SETTING = {"K": K, "eps": 0.04}               # section 9.3 stage B (the gate: these laps only)
LEARN = {"pb": "lmpc", "pb_nominal": "lmpc_nominal", "lmpc": "lmpc"}
FIXED_KINDS = ("nmpc", "identified")
PRIMARY_KINDS = ("nmpc", "identified", "pb", "pb_nominal")
KINDS = PRIMARY_KINDS + ("lmpc",)
SECONDARY = ("IT_LAG30", "IT_WET85")


def setting_key(setting: dict) -> str:
    return cx_a.setting_key(setting)


def cases(name: str) -> list[dict]:
    from .l2 import sample_plants
    from .l8 import wet

    if name in ("NOM", "DEV"):
        return cx_e.cases(name)
    src = {"IV": (301, 10), "IT": (IT_SEED, 50), "IT_LAG30": (IT_SEED, 50), "IT_WET85": (IT_SEED, 50)}
    if name not in src:
        raise KeyError(name)
    seed, n = src[name]
    out = []
    for i, (info, plant) in enumerate(sample_plants(DynVehicle(), n, seed)[1:], start=1):
        kw: dict = {}
        if name == "IT_LAG30":
            kw["tau"] = cx_a.LAG_TAU
        elif name == "IT_WET85":
            kw["wet"], plant = 0.85, wet(plant, 0.85)
        out.append(cx_d._case(name, f"{name}_car{i}", i, info, plant, f"seed{seed}_car{i}", **kw))
    return out


def case_by_name(name: str) -> dict:
    return next(c for c in cases(name.rsplit("_", 1)[0]) if c["name"] == name)


def learn(case: dict, kind: str, setting: dict) -> dict:
    """The initialisation laps and the ten learning laps of one car (phase E's loop): ``pb`` / ``pb_nominal`` need ``eps``
    in the setting (the bound), ``lmpc`` has none (phase E's problem)."""
    if (kind == "lmpc") == ("eps" in setting):
        raise ValueError(f"{kind}: the setting {setting} does not fit")
    r = cx_e.learn(case, LEARN[kind], setting)
    r["kind"] = kind
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
    """Section 4 with clarification n1: phase E's rule for ``pb`` over eps (most completed tenth laps; ties by the mean
    tenth lap on the common cars; then eps = 0.04, then the smaller eps)."""
    return cx_e.select(records, kind="pb", prefer=lambda s: (s["eps"] != 0.04, s["eps"]))


__all__ = ["cases", "case_by_name", "learn", "fixed_lap", "run", "select", "KINDS", "PRIMARY_KINDS", "LEARN", "EPS_GRID",
           "DEV_SETTING", "SECONDARY", "setting_key"]
