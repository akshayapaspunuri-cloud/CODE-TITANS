"""Baseline: nearest neighbour, then 2-opt inside each route and single-village
relocation between routes, first improvement, until neither helps."""

from adapter import Solver, route_load
from adapters.starter import StarterSolver
from data import distance_matrix

SAFETY_S = 0.2


def two_opt(d, route):
    """Reverse segments while that shortens the route (first improvement)."""
    improved = True
    while improved:
        improved = False
        for i in range(len(route) - 1):
            a = route[i - 1] if i else 0
            for j in range(i + 1, len(route)):
                b = route[j + 1] if j + 1 < len(route) else 0
                if d[a][route[j]] + d[route[i]][b] < d[a][route[i]] + d[route[j]][b]:
                    route[i:j + 1] = reversed(route[i:j + 1])
                    improved = True
                    break
            if improved:
                break
    return route


def relocate_once(d, instance, routes, loads):
    """Move one village to the cheapest spot in another route, if that shortens the plan."""
    for a, ra in enumerate(routes):
        for i, v in enumerate(ra):
            p = ra[i - 1] if i else 0
            q = ra[i + 1] if i + 1 < len(ra) else 0
            saving = d[p][v] + d[v][q] - d[p][q]
            for b, rb in enumerate(routes):
                if b == a or loads[b] + instance.demand[v] > instance.capacity:
                    continue
                for j in range(len(rb) + 1):
                    x = rb[j - 1] if j else 0
                    y = rb[j] if j < len(rb) else 0
                    if d[x][v] + d[v][y] - d[x][y] < saving:
                        ra.pop(i)
                        rb.insert(j, v)
                        loads[a] -= instance.demand[v]
                        loads[b] += instance.demand[v]
                        return True
    return False


class BaselineSolver(Solver):
    def solve(self, instance, submit_candidate):
        d = distance_matrix(instance)
        routes = StarterSolver().solve(instance, submit_candidate)["routes"]
        receipt = submit_candidate({"routes": routes})
        loads = [route_load(instance, r) for r in routes]
        while receipt["remaining_s"] > SAFETY_S:
            for r in routes:
                two_opt(d, r)
            if not relocate_once(d, instance, routes, loads):
                break
            receipt = submit_candidate({"routes": routes})
        return {"routes": [r for r in routes if r]}
