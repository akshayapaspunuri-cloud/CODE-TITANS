# Tanker Run

Route water tankers from one depot to every village without splitting a delivery.

## The scenario

Ravi Deshmukh coordinates the dry-season water run out of Pimpalgaon. Each
village has a fixed number of kilolitres booked, each tanker holds the same
amount, and the district owns a fixed number of tankers. Diesel is the
budget: the shorter the total driving, the more water next week.

## The problem, precisely

- **Places:** depot `0` and villages `1..n` (`n = instance.size`) at
  `instance.coords[i] = (x, y)` on a 0..1000 grid.
- **Distance:** `isqrt(dx^2 + dy^2)`, symmetric.
- **Demand:** `instance.demand[v]` for each village (the depot's is 0).
- **Tankers:** capacity `instance.capacity`; at most `instance.fleet`
  non-empty routes.
- **Plan:** `{"routes": [[v, v, ...], ...]}`. The depot is implicit at both
  ends of every route. Every village exactly once; each route's total demand
  at most the capacity. Empty routes are allowed and ignored.
- **Cost:** total distance driven.

## How you're scored

Your best valid cost is read at 5%, 20%, 50% and 100% of the 6-second budget
(weights 0.10, 0.20, 0.30, 0.40). At each checkpoint the cost is placed on a
curve through three anchors: the starter scores 0.25, the published baseline
0.50, the reference 1.00, linearly in between; beating the reference scores
1.00, no valid plan scores 0.

*Example:* baseline 6,970, reference 6,325 at a checkpoint. A cost of 6,648 is
halfway between them and scores 0.75 there.

Points: quality 70, robustness 20 (shifted instances, halved if one family is
strong and another weak), engineering 10.

## Your submission

```python
from adapter import Solver

class MySolver(Solver):
    def solve(self, instance, submit_candidate):
        receipt = submit_candidate({"routes": [...]})
        return {"routes": [...]}
```

Put it in `adapters/mine.py`, then:

```
python self_check.py --adapter adapters.mine:MySolver
python run.py --adapter adapters.mine:MySolver --out report.json
```

## The trap

Read this before you write anything: **a village's demand cannot be split.**
A plan that lists a village in two routes is rejected, however convenient.

## Baselines

| Anchor | Program | Score |
| --- | --- | --- |
| Starter | nearest neighbour: drive to the nearest village that still fits | 0.25 |
| Baseline | nearest neighbour, then 2-opt in each route and moving single villages between routes | 0.50 |
| Reference | a stronger search, not published | 1.00 |

## Your head start

`adapters/starter.py` is a working solver. `adapter.py` gives you
`dist(instance, a, b)`, `route_load(instance, route)`,
`route_length(instance, route)` and `total_length(instance, routes)`.

## A hint

Improving each route on its own plateaus; the next gains come from moving villages between routes.

## Instance families

| Family | Public | Private | What changes |
| --- | --- | --- | --- |
| uniform | 4 | 4 | villages spread evenly, loose capacity (~75% full), depot central |
| clustered-tight (shifted) | 2 | 4 | clustered villages, tight capacity (~92% full), depot on an edge |

Sizes: 30 to 150 villages. The private suite uses unseen seeds.

## Files

`adapter.py`, `adapters/starter.py`, `adapters/baseline.py`, `data.py`,
`validator.py`, `self_check.py`, `run.py`, `public_reference.json`,
`benchkit/`, `SECURITY.md`.
