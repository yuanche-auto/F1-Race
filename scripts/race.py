"""Your first race: three drivers, one flying lap, a classification and a replay.

    python scripts/race.py                                # Spielberg, the three drivers, grip 0.97
    python scripts/race.py --grip 1.0                     # the plans use all of the grip
    python scripts/race.py --drivers ocp centre           # pick the field
    python scripts/race.py --track Monza                  # another circuit (downloaded on first use)

Drivers (src/f1race/race.py): ocp – the minimum-time OCP; mincurv – the minimum-curvature line
with a quasi-steady speed profile; centre – the centre line with a quasi-steady speed profile.
Each plan is driven by the L0 tracker on the kinematic car in the simulator. The first race on
a track takes about a minute (the plans); the plans are cached, so the next one takes seconds.
The replay is written to results/race/<track>_grip<grip>.html – open it in a browser.
"""
from __future__ import annotations

import argparse
import sys
import time
import webbrowser
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

from fetch_tracks import SHA256, fetch  # noqa: E402

from f1race.race import DRIVERS, RaceConfig, race, report, write_replay  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--track", default="Spielberg", choices=sorted(SHA256), help="circuit (default: Spielberg)")
    ap.add_argument("--drivers", nargs="+", default=list(DRIVERS), choices=list(DRIVERS), help="the field")
    ap.add_argument("--grip", type=float, default=0.97,
                    help="share of the friction ellipse the plans use, 0.5-1.0 (default 0.97, as in L0)")
    ap.add_argument("--margin", type=float, default=0.3, help="planned distance to the track edges [m] (default 0.3)")
    ap.add_argument("--no-cache", action="store_true", help="recompute the plans")
    ap.add_argument("--replay", type=Path, default=None, help="where to write the HTML replay")
    ap.add_argument("--open", action="store_true", help="open the replay in the browser")
    args = ap.parse_args(argv)
    if not 0.5 <= args.grip <= 1.0:
        ap.error("--grip must be between 0.5 and 1.0")

    csv = ROOT / "data" / "tracks" / f"{args.track}.csv"
    if not csv.exists():
        print(f"Downloading {args.track} from the TUM racetrack-database (pinned commit, SHA-256 checked) ...")
    fetch(args.track)
    n = len(args.drivers)
    print(f"F1-Race | {args.track} | {n} driver{'s' if n != 1 else ''} | one flying lap | kinematic car | "
          f"grip used by the plans: {args.grip:.0%}\n")
    t0 = time.time()
    cfg = RaceConfig(track=args.track, drivers=tuple(args.drivers), grip=args.grip, margin=args.margin,
                     cache=not args.no_cache)
    res = race(cfg)
    print()
    print(report(res))
    out = args.replay or ROOT / "results" / "race" / f"{args.track}_grip{args.grip:.2f}.html"
    out = write_replay(res, out)
    try:
        shown = out.resolve().relative_to(Path.cwd().resolve())
    except ValueError:
        shown = out
    print(f"\nReplay: {shown}   (open it in a browser)   - {time.time() - t0:.0f} s in all")
    if args.open:
        webbrowser.open(out.resolve().as_uri())
    return 0


if __name__ == "__main__":
    sys.exit(main())
