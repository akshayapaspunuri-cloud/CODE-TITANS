"""Tanker Run: capacitated vehicle routing.

Depot 0 and villages 1..n on a 0..1000 grid. Villages are first dealt into m
balanced groups; only then are the capacity Q and the fleet size K fixed, so
those groups are always a feasible plan. K leaves 20% slack over m. Each suite
entry names the draw (profile["attempt"]) to use; the organizer's
private/find_attempts.py picks attempts where the starter fits within K. This
file does exactly one draw. Integer-only and deterministic.
"""

from dataclasses import dataclass
from math import isqrt
from types import MappingProxyType

from benchkit import SuiteEntry, freeze_profile
from benchkit.rng import Rng, derive_seed, digest_ints

SCHEMA_VERSION = 1
GRID = 1000
FLEET_SLACK_PCT = 20

UNIFORM = {"family": "uniform", "shifted": False, "layout": "uniform", "depot": "central",
           "fill_pct": 75, "per_route_lo": 8, "per_route_hi": 12, "demand_lo": 5, "demand_hi": 30}
CLUSTERED = {"family": "clustered-tight", "shifted": True, "layout": "clustered", "depot": "edge",
             "fill_pct": 92, "per_route_lo": 6, "per_route_hi": 10, "demand_lo": 5, "demand_hi": 40}


@dataclass(frozen=True)
class Instance:
    name: str
    profile: MappingProxyType
    size: int          # number of villages n (ids 1..n; 0 is the depot)
    digest: str
    coords: tuple      # ((x, y), ...) for ids 0..n
    demand: tuple      # demand for ids 0..n (the depot's is 0)
    capacity: int      # Q
    fleet: int         # K: at most this many non-empty routes


def compute_digest(instance) -> str:
    return digest_ints((instance.size, instance.capacity, instance.fleet, *instance.demand,
                        *(c for xy in instance.coords for c in xy)), instance.profile)


def _dist_matrix(coords):
    return [[isqrt((ax - bx) ** 2 + (ay - by) ** 2) for bx, by in coords] for ax, ay in coords]


_MATRICES = {}


def distance_matrix(instance):
    """d[a][b] = isqrt(dx^2 + dy^2) for ids 0..n. Built once per instance; treat as read-only."""
    digest = instance.digest
    d = _MATRICES.get(digest)
    if d is None:
        d = _dist_matrix(instance.coords)
        _MATRICES[digest] = d
    return d


def clear_matrix_cache():
    """Utility to prevent memory exhaustion in multi-run harness environments."""
    _MATRICES.clear()


def draw(seed, profile):
    """One draw: (instance fields, the m balanced village groups Q and K were set from)."""
    profile = freeze_profile(profile)
    rng = Rng(derive_seed("tanker", SCHEMA_VERSION, seed, profile["attempt"]))
    n = profile["n"]

    if profile["depot"] == "edge":
        side, along = rng.below(4), rng.between(0, GRID)
        depot = ((along, 0), (along, GRID), (0, along), (GRID, along))[side]
    else:
        depot = (rng.between(450, 550), rng.between(450, 550))

    clusters = []
    if profile["layout"] == "clustered":
        clusters = [(rng.between(100, 900), rng.between(100, 900), rng.between(40, 100))
                    for _ in range(rng.between(3, 6))]
    villages = []
    for _ in range(n):
        if clusters and rng.chance(80, 100):
            cx, cy, spread = rng.choice(clusters)
            villages.append((max(0, min(GRID, cx + rng.between(-spread, spread))),
                             max(0, min(GRID, cy + rng.between(-spread, spread)))))
        else:
            villages.append((rng.between(0, GRID), rng.between(0, GRID)))
    demand = [0] + [rng.between(profile["demand_lo"], profile["demand_hi"]) for _ in range(n)]

    # Deal villages into m balanced groups (largest demand to the lightest group).
    m = max(2, n // rng.between(profile["per_route_lo"], profile["per_route_hi"]))
    groups, loads = [[] for _ in range(m)], [0] * m
    for v in sorted(range(1, n + 1), key=lambda v: (-demand[v], v)):
        g = min(range(m), key=lambda g: (loads[g], g))
        groups[g].append(v)
        loads[g] += demand[v]
    total = sum(demand)
    capacity = max(max(loads), -(-total * 100 // (m * profile["fill_pct"])))
    fleet = m + -(-m * FLEET_SLACK_PCT // 100)
    fields = dict(profile=profile, size=n, coords=(depot, *villages), demand=tuple(demand),
                  capacity=capacity, fleet=fleet)
    return fields, groups


def make_instance(seed, profile, name) -> Instance:
    fields, _ = draw(seed, profile)
    return Instance(name=name, digest=compute_digest(Instance(name=name, digest="", **fields)), **fields)


def _entry(name, seed, family, n, attempt):
    return SuiteEntry(name, seed, {**family, "n": n, "attempt": attempt})


PUBLIC_SUITE = (
    _entry("tanker-01", 4101, UNIFORM, 40, 0),
    _entry("tanker-02", 4102, UNIFORM, 70, 0),
    _entry("tanker-03", 4103, UNIFORM, 110, 0),
    _entry("tanker-04", 4104, UNIFORM, 150, 0),
    _entry("tanker-05", 4105, CLUSTERED, 60, 0),
    _entry("tanker-06", 4106, CLUSTERED, 120, 0),
)
SELF_CHECK_NAMES = ("tanker-01", "tanker-05")


def budget_for(instance) -> float:
    return 6.0