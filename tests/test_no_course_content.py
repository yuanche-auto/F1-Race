"""Guard: the repository must not contain material from a specific course package.

Only SHA-256 hashes of the banned (lower-case) tokens are stored here, so this
file does not itself contain the terms. Every tracked text file is tokenised;
single tokens and concatenated neighbouring token pairs are hashed and checked.
"""
import hashlib
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

BANNED_SHA256 = {
    "006410180bb76ec4229acd875d4c986c5c8cd4173f1c96a6dee5e393ffbd165d",
    "05f5c29c316a61df5e8485b3340f2656dcd42eb54fbf69724af14dd366e1d740",
    "5518930449383889366d4253b29e15a6eeab8df437dc4a5d47f6531de71bcd77",
    "6cd77d23be087d49a7ad414ea8926dc322f1e2766362a5b3c76dc53179f86589",
    "723a15e4893624c2a91f7f2a97f1723b9d5dc7f6766c57822fb07b420cf90956",
    "7b502e76e11b481f47dd2daa9cb75f8c9f711a5c3d3cb7f6b62a99581a9bee8b",
    "7f6a5b2ff3a3905086d2f3f950d697ca5275f0ab374ff9d84525c1cfa363e3ea",
    "ba82dc203851fda91217262c00396dcfec249e125df02c2ac195d29498fdc12b",
    "c0b8645dde83c048b2e0b4b649f066a8d9d33c86ff34b0116bf265f003c9dd80",
    "c57dab960048b274dc842acefb0a6214518b3240378732cc183fd970113bcc91",
    "cf502ac6410fdbaea492a2bfc12f05274eb1d5415440494a09395085c470856c",
    "dca42bbd974c580b946a1dacc7e673883cf93e16d764b7e83d354a89affb716e",
    "e80b15c7a3cd5d125d7f184dcaa6ca5300fc905d48d956570493617d0e521e85",
    "ef88bf6d9fbd0a5ca075da9b1cb8eab858a9d666fff0d59275005b0181e7aaa9",
}

TEXT_SUFFIXES = {".py", ".md", ".toml", ".txt", ".cfg", ".ini", ".yml", ".yaml", ".json", ".m", ""}


def tracked_files():
    try:
        out = subprocess.run(["git", "ls-files"], cwd=ROOT, capture_output=True, text=True, check=True).stdout
        files = [ROOT / f for f in out.splitlines() if f]
    except Exception:
        files = []
    # also scan untracked source files that would be committed
    for sub in ("src", "scripts", "tests", "docs"):
        files += [p for p in (ROOT / sub).rglob("*") if p.is_file()]
    files += [p for p in ROOT.glob("*") if p.is_file()]
    return sorted({f for f in files if f.exists()})


def test_no_course_tokens():
    hits = []
    for f in tracked_files():
        if f.suffix.lower() not in TEXT_SUFFIXES:
            continue
        text = f.read_text(encoding="utf-8", errors="ignore").lower()
        toks = re.findall(r"[a-z0-9_]+", text)
        cands = set(toks) | {a + b for a, b in zip(toks, toks[1:])}
        for c in cands:
            if hashlib.sha256(c.encode()).hexdigest() in BANNED_SHA256:
                hits.append(f.relative_to(ROOT).as_posix())
                break
    assert not hits, f"banned course-specific tokens found in: {hits}"


def test_no_data_files_tracked():
    out = subprocess.run(["git", "ls-files"], cwd=ROOT, capture_output=True, text=True).stdout.split()
    bad = [f for f in out if f.lower().endswith((".csv", ".mat", ".npz", ".pkl")) or f.startswith("data/")]
    assert not bad, f"data files must not be committed: {bad}"
