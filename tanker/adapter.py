"""The interface every Tanker Run solver implements, plus helpers.

A plan is {"routes": [[v, v, ...], ...]}: village ids 1..n, the depot (0)
implicit at both ends of every route.
"""

from abc import ABC, abstractmethod

from data import distance_matrix


class Solver(ABC):
    @abstractmethod
    def solve(self, instance, submit_candidate):
        """Call submit_candidate(plan) any number of times; each call returns a receipt
        (accepted, reason, cost, best, elapsed_s, remaining_s). The return value is one
        more candidate."""


def dist(instance, a, b) -> int:
    """Distance between ids a and b (0 is the depot), as the evaluator measures it."""
    return distance_matrix(instance)[a][b]


def route_load(instance, route) -> int:
    return sum(instance.demand[v] for v in route)


def route_length(instance, route) -> int:
    """Length of depot -> route -> depot (0 for an empty route)."""
    if not route:
        return 0
    d = distance_matrix(instance)
    cost = d[0][route[0]]
    for i in range(len(route) - 1):
        cost += d[route[i]][route[i + 1]]
    cost += d[route[-1]][0]
    return cost


def total_length(instance, routes) -> int:
    d = distance_matrix(instance)
    total = 0
    for r in routes:
        if not r:
            continue
        total += d[0][r[0]]
        for i in range(len(r) - 1):
            total += d[r[i]][r[i + 1]]
        total += d[r[-1]][0]
    return total
