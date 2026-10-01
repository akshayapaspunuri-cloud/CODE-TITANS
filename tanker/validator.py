"""Feasibility and canonical cost for Tanker Run.

Every village exactly once, each route's load within capacity, at most K
non-empty routes; the depot is implicit at both ends. cost = total distance.
"""

from data import distance_matrix


def validate(instance, candidate):
    if not isinstance(candidate, dict):
        return None, "candidate must be a dict"
    routes = candidate.get("routes")
    if not isinstance(routes, list):
        return None, "'routes' must be a list of lists of village ids"
    
    n = instance.size
    if len(routes) > max(n, instance.fleet):
        return None, f"{len(routes)} routes listed, more than there could ever be"
    
    total = 0
    for route in routes:
        if not isinstance(route, list):
            return None, "every route must be a list of village ids"
        total += len(route)
        if total > n:
            return None, f"more than {n} village ids in the plan (a village cannot be split)"
    
    if total < n:
        return None, f"only {total} of {n} villages are visited"

    d = distance_matrix(instance)
    seen = bytearray(n + 1)
    used = 0
    length = 0

    for k, route in enumerate(routes):
        if not route:
            continue
        load = 0
        prev = 0
        for v in route:
            # Handle int and numpy/int-like types safely if within range
            if not isinstance(v, (int,)) or isinstance(v, bool) or not (1 <= v <= n):
                return None, f"village id {v!r} is not an integer in 1..{n}"
            if seen[v]:
                return None, f"village {v} is visited twice (demand cannot be split)"
            seen[v] = 1
            load += instance.demand[v]
            length += d[prev][v]
            prev = v
        length += d[prev][0]
        if load > instance.capacity:
            return None, f"route {k} carries {load} > capacity {instance.capacity}"
        used += 1

    if used > instance.fleet:
        return None, f"{used} tankers used, the fleet has {instance.fleet}"
    
    return length, None