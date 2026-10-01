"""High-performance CVRP solver for Tanker Run.

Tuned for near-reference costs across all instances:
- Spatial K-Center + Multi-offset Angular Sweep + Parametric Clarke-Wright.
- Fast O(1) intra-route 2-Opt.
- 2-Opt* (inter-route head/tail exchanges with reversals).
- Node Relocation & Pair Relocation (Or-Opt).
- Pairwise Cross-Exchange (Swap).
- Paced checkpoint submissions.
"""

import math
import random
import time
from adapter import Solver, dist


class MySolver(Solver):
    def solve(self, instance, submit_candidate):
        t0 = time.perf_counter()
        n = instance.size
        coords = instance.coords
        demands = instance.demand
        capacity = instance.capacity
        fleet = instance.fleet

        # Fast direct-lookup table
        d = [[dist(instance, i, j) for j in range(n + 1)] for i in range(n + 1)]

        def r_len(route):
            if not route:
                return 0
            c = d[0][route[0]]
            for k in range(len(route) - 1):
                c += d[route[k]][route[k + 1]]
            return c + d[route[-1]][0]

        def total_cost(routes):
            return sum(r_len(r) for r in routes if r)

        def two_opt(route):
            if len(route) < 3:
                return route
            best = list(route)
            improved = True
            while improved:
                improved = False
                n_r = len(best)
                for i in range(n_r - 1):
                    for j in range(i + 1, n_r):
                        pi = 0 if i == 0 else best[i - 1]
                        nj = 0 if j == n_r - 1 else best[j + 1]
                        cur = d[pi][best[i]] + d[best[j]][nj]
                        cand = d[pi][best[j]] + d[best[i]][nj]
                        if cand < cur:
                            best = best[:i] + best[i : j + 1][::-1] + best[j + 1 :]
                            improved = True
                            break
                    if improved:
                        break
            return best

        def fit_fleet(routes_in):
            res = [list(r) for r in routes_in if r]
            while len(res) > fleet:
                idx = min(range(len(res)), key=lambda i: sum(demands[v] for v in res[i]))
                orphan = res.pop(idx)
                for node in orphan:
                    placed = False
                    for r in res:
                        if sum(demands[x] for x in r) + demands[node] <= capacity:
                            r.append(node)
                            placed = True
                            break
                    if not placed:
                        res.append([node])
                        break
            return res

        candidates = []

        # ---------------------------------------------------------
        # 1. K-Center Clustering (Clustered & Edge instances)
        # ---------------------------------------------------------
        seeds = [1]
        for _ in range(1, min(fleet, n)):
            best_cand, best_min_dist = -1, -1
            for v in range(1, n + 1):
                if v in seeds:
                    continue
                min_d = min(d[v][s] for s in seeds)
                if min_d > best_min_dist:
                    best_min_dist = min_d
                    best_cand = v
            if best_cand != -1:
                seeds.append(best_cand)

        hub_routes = [[] for _ in seeds]
        hub_loads = [0] * len(seeds)
        remaining = sorted(range(1, n + 1), key=lambda v: min(d[v][s] for s in seeds))

        for v in remaining:
            sorted_hubs = sorted(range(len(seeds)), key=lambda h: d[v][seeds[h]])
            placed = False
            for h in sorted_hubs:
                if hub_loads[h] + demands[v] <= capacity:
                    hub_routes[h].append(v)
                    hub_loads[h] += demands[v]
                    placed = True
                    break
            if not placed:
                hub_routes.append([v])
                hub_loads.append(demands[v])

        cand_hub = fit_fleet(hub_routes)
        if len(cand_hub) <= fleet:
            candidates.append([two_opt(r) for r in cand_hub])

        # ---------------------------------------------------------
        # 2. Polar Sweep Constructions
        # ---------------------------------------------------------
        depot_x, depot_y = coords[0]
        angles = {v: math.atan2(coords[v][1] - depot_y, coords[v][0] - depot_x) for v in range(1, n + 1)}

        for offset in (0.0, 0.6, 1.2, 1.8, 2.4, 3.0):
            sorted_v = sorted(range(1, n + 1), key=lambda v: (angles[v] + offset) % (2 * math.pi))
            sw_routes, cur_r, cur_l = [], [], 0
            for v in sorted_v:
                if cur_l + demands[v] <= capacity:
                    cur_r.append(v)
                    cur_l += demands[v]
                else:
                    if cur_r:
                        sw_routes.append(cur_r)
                    cur_r, cur_l = [v], demands[v]
            if cur_r:
                sw_routes.append(cur_r)
            sw_fit = fit_fleet(sw_routes)
            if len(sw_fit) <= fleet:
                candidates.append([two_opt(r) for r in sw_fit])

        # ---------------------------------------------------------
        # 3. Parametric Clarke-Wright Savings
        # ---------------------------------------------------------
        for gamma in (1.0, 0.75, 1.25):
            cw_routes = [[v] for v in range(1, n + 1)]
            savings = []
            for i in range(1, n + 1):
                for j in range(i + 1, n + 1):
                    savings.append((d[i][0] + d[0][j] - gamma * d[i][j], i, j))
            savings.sort(reverse=True, key=lambda x: x[0])

            node_to_r = {v: i for i, v in enumerate(range(1, n + 1))}
            r_loads = [demands[v] for v in range(1, n + 1)]

            for s, u, v in savings:
                ru, rv = node_to_r[u], node_to_r[v]
                if ru == rv:
                    continue
                if r_loads[ru] + r_loads[rv] <= capacity:
                    r1, r2 = cw_routes[ru], cw_routes[rv]
                    if r1[-1] == u and r2[0] == v:
                        new_r = r1 + r2
                    elif r2[-1] == v and r1[0] == u:
                        new_r = r2 + r1
                    elif r1[0] == u and r2[0] == v:
                        new_r = r1[::-1] + r2
                    elif r1[-1] == u and r2[-1] == v:
                        new_r = r1 + r2[::-1]
                    else:
                        continue
                    cw_routes[ru] = new_r
                    r_loads[ru] += r_loads[rv]
                    cw_routes[rv] = []
                    r_loads[rv] = 0
                    for node in new_r:
                        node_to_r[node] = ru

            cw_fit = fit_fleet([r for r in cw_routes if r])
            if len(cw_fit) <= fleet:
                candidates.append([two_opt(r) for r in cw_fit])

        # Best starter pick - checkpoint 1 submission (< 0.15s)
        best_routes = min(candidates, key=total_cost)
        submit_candidate({"routes": [list(r) for r in best_routes]})

        # Fast preliminary 2-opt (< 0.25s)
        for i in range(len(best_routes)):
            best_routes[i] = two_opt(best_routes[i])
        submit_candidate({"routes": [list(r) for r in best_routes]})

        # ---------------------------------------------------------
        # 4. Multi-Neighborhood Local Search
        # ---------------------------------------------------------
        deadline = 5.68
        rng = random.Random(1337)

        while time.perf_counter() - t0 < deadline:
            improved = False

            # Move A: 2-Opt* (Inter-route edge break)
            for r1_idx in range(len(best_routes)):
                for r2_idx in range(r1_idx + 1, len(best_routes)):
                    r1, r2 = best_routes[r1_idx], best_routes[r2_idx]
                    base_c = r_len(r1) + r_len(r2)

                    for i in range(1, len(r1)):
                        head1, tail1 = r1[:i], r1[i:]
                        w_head1 = sum(demands[v] for v in head1)
                        w_tail1 = sum(demands[v] for v in tail1)

                        for j in range(1, len(r2)):
                            head2, tail2 = r2[:j], r2[j:]
                            w_head2 = sum(demands[v] for v in head2)
                            w_tail2 = sum(demands[v] for v in tail2)

                            # Case 1: head1 + tail2, head2 + tail1
                            if (w_head1 + w_tail2 <= capacity) and (w_head2 + w_tail1 <= capacity):
                                cand1 = head1 + tail2
                                cand2 = head2 + tail1
                                if r_len(cand1) + r_len(cand2) < base_c:
                                    best_routes[r1_idx] = two_opt(cand1)
                                    best_routes[r2_idx] = two_opt(cand2)
                                    submit_candidate({"routes": [list(r) for r in best_routes]})
                                    improved = True
                                    break

                            # Case 2: head1 + reversed(head2), reversed(tail1) + tail2
                            if not improved and (w_head1 + w_head2 <= capacity) and (w_tail1 + w_tail2 <= capacity):
                                cand1 = head1 + head2[::-1]
                                cand2 = tail1[::-1] + tail2
                                if r_len(cand1) + r_len(cand2) < base_c:
                                    best_routes[r1_idx] = two_opt(cand1)
                                    best_routes[r2_idx] = two_opt(cand2)
                                    submit_candidate({"routes": [list(r) for r in best_routes]})
                                    improved = True
                                    break

                        if improved or (time.perf_counter() - t0 >= deadline):
                            break
                    if improved or (time.perf_counter() - t0 >= deadline):
                        break
                if improved or (time.perf_counter() - t0 >= deadline):
                    break

            # Move B: Node Swap (Pairwise cross exchange)
            if not improved and (time.perf_counter() - t0 < deadline):
                for r1_idx in range(len(best_routes)):
                    for r2_idx in range(r1_idx + 1, len(best_routes)):
                        r1, r2 = best_routes[r1_idx], best_routes[r2_idx]
                        l1, l2 = sum(demands[v] for v in r1), sum(demands[v] for v in r2)

                        for i, u in enumerate(r1):
                            for j, v in enumerate(r2):
                                if (l1 - demands[u] + demands[v] <= capacity) and (
                                    l2 - demands[v] + demands[u] <= capacity
                                ):
                                    base_c = r_len(r1) + r_len(r2)
                                    tr1 = r1[:i] + [v] + r1[i + 1 :]
                                    tr2 = r2[:j] + [u] + r2[j + 1 :]
                                    if r_len(tr1) + r_len(tr2) < base_c:
                                        best_routes[r1_idx] = two_opt(tr1)
                                        best_routes[r2_idx] = two_opt(tr2)
                                        submit_candidate({"routes": [list(r) for r in best_routes]})
                                        improved = True
                                        break
                            if improved or (time.perf_counter() - t0 >= deadline):
                                break
                        if improved or (time.perf_counter() - t0 >= deadline):
                            break
                    if improved or (time.perf_counter() - t0 >= deadline):
                        break

            # Move C: Relocate Single Node
            if not improved and (time.perf_counter() - t0 < deadline):
                for r1_idx in range(len(best_routes)):
                    for r2_idx in range(len(best_routes)):
                        if r1_idx == r2_idx:
                            continue
                        r1, r2 = best_routes[r1_idx], best_routes[r2_idx]
                        l2 = sum(demands[v] for v in r2)

                        for i, u in enumerate(r1):
                            if l2 + demands[u] <= capacity:
                                base_c = r_len(r1) + r_len(r2)
                                rem_r1 = r1[:i] + r1[i + 1 :]

                                best_ins_c, best_r2 = 10**9, None
                                for pos in range(len(r2) + 1):
                                    cand = r2[:pos] + [u] + r2[pos:]
                                    c = r_len(cand)
                                    if c < best_ins_c:
                                        best_ins_c = c
                                        best_r2 = cand

                                if r_len(rem_r1) + best_ins_c < base_c:
                                    best_routes[r1_idx] = two_opt(rem_r1)
                                    best_routes[r2_idx] = two_opt(best_r2)
                                    best_routes = [r for r in best_routes if r]
                                    submit_candidate({"routes": [list(r) for r in best_routes]})
                                    improved = True
                                    break
                        if improved or (time.perf_counter() - t0 >= deadline):
                            break
                    if improved or (time.perf_counter() - t0 >= deadline):
                        break

            # Move D: Relocate Pair (Or-Opt consecutive 2 nodes)
            if not improved and (time.perf_counter() - t0 < deadline):
                for r1_idx in range(len(best_routes)):
                    for r2_idx in range(len(best_routes)):
                        if r1_idx == r2_idx or len(best_routes[r1_idx]) < 3:
                            continue
                        r1, r2 = best_routes[r1_idx], best_routes[r2_idx]
                        l2 = sum(demands[v] for v in r2)

                        for i in range(len(r1) - 1):
                            u, v = r1[i], r1[i + 1]
                            load_pair = demands[u] + demands[v]
                            if l2 + load_pair <= capacity:
                                base_c = r_len(r1) + r_len(r2)
                                rem_r1 = r1[:i] + r1[i + 2 :]

                                best_ins_c, best_r2 = 10**9, None
                                for pos in range(len(r2) + 1):
                                    cand = r2[:pos] + [u, v] + r2[pos:]
                                    c = r_len(cand)
                                    if c < best_ins_c:
                                        best_ins_c = c
                                        best_r2 = cand

                                if r_len(rem_r1) + best_ins_c < base_c:
                                    best_routes[r1_idx] = two_opt(rem_r1)
                                    best_routes[r2_idx] = two_opt(best_r2)
                                    best_routes = [r for r in best_routes if r]
                                    submit_candidate({"routes": [list(r) for r in best_routes]})
                                    improved = True
                                    break
                        if improved or (time.perf_counter() - t0 >= deadline):
                            break
                    if improved or (time.perf_counter() - t0 >= deadline):
                        break

            # Perturbation agar search plateau par atak jaye
            if not improved and (time.perf_counter() - t0 < deadline - 0.8):
                if len(best_routes) >= 2:
                    ra = rng.randint(0, len(best_routes) - 1)
                    if best_routes[ra]:
                        val = best_routes[ra].pop(rng.randint(0, len(best_routes[ra]) - 1))
                        inserted = False
                        for rb in range(len(best_routes)):
                            if rb != ra and sum(demands[x] for x in best_routes[rb]) + demands[val] <= capacity:
                                best_routes[rb].append(val)
                                best_routes[rb] = two_opt(best_routes[rb])
                                inserted = True
                                break
                        if not inserted:
                            best_routes[ra].append(val)
                        else:
                            best_routes = [r for r in best_routes if r]
                            submit_candidate({"routes": [list(r) for r in best_routes]})
            elif not improved:
                break

        return {"routes": [list(r) for r in best_routes if r]}