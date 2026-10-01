"""Puts this bench and the nearest benchkit/ on sys.path."""

import sys
from pathlib import Path

BENCH_DIR = Path(__file__).resolve().parent
_root_candidates = [p for p in (BENCH_DIR, *BENCH_DIR.parents) if (p / "benchkit").is_dir()]
ROOT = _root_candidates[0] if _root_candidates else None

for _p in (ROOT, BENCH_DIR):
    if _p and str(_p) not in sys.path:
        sys.path.insert(0, str(_p))
