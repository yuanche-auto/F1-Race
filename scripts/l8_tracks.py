"""L8: the draw of the validation and test tracks (docs/L8_protocol.md section 2).

    python scripts/l8_tracks.py

The 25 tracks of the TUM FTM racetrack-database at the pinned commit (scripts/fetch_tracks.py),
without Spielberg (L0-L7) and IMS (an oval: no corner sequence of the kind the other tracks
have; excluded before any track was loaded). The 23 others, sorted by name, are permuted with
numpy's default_rng(8) (8 for L8): the first two are the validation tracks, the next four the
test tracks, the rest the reserve, in that order (a test track whose plans cannot be computed is
replaced by the next reserve track, section 2). The draw depends only on this list and the seed.
"""
from __future__ import annotations

import json

import numpy as np

DATABASE = ["Austin", "BrandsHatch", "Budapest", "Catalunya", "Hockenheim", "IMS", "Melbourne", "MexicoCity",
            "Montreal", "Monza", "MoscowRaceway", "Norisring", "Nuerburgring", "Oschersleben", "Sakhir", "SaoPaulo",
            "Sepang", "Shanghai", "Silverstone", "Sochi", "Spa", "Spielberg", "Suzuka", "YasMarina", "Zandvoort"]
EXCLUDED = {"Spielberg": "the track of L0-L7", "IMS": "oval"}
SEED = 8


def draw() -> dict:
    eligible = sorted(t for t in DATABASE if t not in EXCLUDED)
    order = [eligible[i] for i in np.random.default_rng(SEED).permutation(len(eligible))]
    return {"validation": order[:2], "test": order[2:6], "reserve": order[6:]}


if __name__ == "__main__":
    print(json.dumps(draw(), indent=1))
