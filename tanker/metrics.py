"""Bench config for Tanker Run."""

import _paths  # noqa: F401

import data
import validator
from benchkit import BenchConfig

BENCH = BenchConfig(
    name="tanker",
    bench_dir=_paths.BENCH_DIR,
    data=data,
    validate=validator.validate,
    shift_markers=lambda p: "clustered, tight capacity, depot on an edge" if p["shifted"] else "",
)
