"""Starter: nearest neighbour. Each tanker drives to the nearest unvisited village that
still fits, and returns to the depot when none fits."""

from adapter import Solver, dist


class StarterSolver(Solver):
    def solve(self, instance, submit_candidate):
        unvisited = set(range(1, instance.size + 1))
        routes = []
        while unvisited:
            route, load, pos = [], 0, 0
            while True:
                fits = [v for v in unvisited if load + instance.demand[v] <= instance.capacity]
                if not fits:
                    break
                v = min(fits, key=lambda v: (dist(instance, pos, v), v))
                route.append(v)
                load += instance.demand[v]
                pos = v
                unvisited.remove(v)
            routes.append(route)
        return {"routes": routes}
