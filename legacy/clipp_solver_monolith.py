#!/usr/bin/env python3
"""
CLIPP MILP Solver using OR-Tools CP-SAT
========================================
Solves the Climate-Integrated Street Cleaning Routing Problem (CLIPP).

Usage:
    python clipp_solver.py <input_file> [--time_limit <seconds>] [--num_workers <cores>] [--output <file>]

Examples:
    python clipp_solver.py "1. Instances/toy.txt" --time_limit 60 --num_workers 4
    python clipp_solver.py "1. Instances/A_instance.txt" --time_limit 300 --num_workers 8 --output solution.txt
"""

import argparse
import collections
import heapq
import sys
import time
from pathlib import Path

from ortools.sat.python import cp_model


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------

def parse_instance(filepath: str) -> dict:
    """
    Parse a CLIPP instance file.

    Supported header formats:
        N M T C S         (W defaults to 0, no junction coordinates)
        N M T C S W       (W given, no junction coordinates)
    Followed by M street lines, then one vehicle-types line.
    """
    with open(filepath) as fh:
        raw = [ln.strip() for ln in fh if ln.strip()]

    idx = 0
    header = raw[idx].split()
    idx += 1

    N = int(header[0])   # junctions
    M = int(header[1])   # streets
    T = int(header[2])   # time limit (seconds)
    C = int(header[3])   # vehicles
    S = int(header[4])   # depot junction
    W = float(header[5]) if len(header) >= 6 else 0.0  # water-waste penalty

    # Detect optional junction-coordinate block:
    # coordinates look like two floats; street lines have 7 tokens.
    junction_coords = []
    if idx < len(raw):
        tokens = raw[idx].split()
        if len(tokens) == 2:
            try:
                float(tokens[0]); float(tokens[1])
                for _ in range(N):
                    parts = raw[idx].split()
                    junction_coords.append((float(parts[0]), float(parts[1])))
                    idx += 1
            except ValueError:
                pass  # tokens are not floats – no coordinate block

    # Parse M street lines: A B D cost length cat req
    CAT_VALID  = {'M', 'O', 'C'}
    streets = []
    for j in range(M):
        parts = raw[idx].split()
        idx += 1
        a     = int(parts[0])
        b     = int(parts[1])
        d     = int(parts[2])   # 1 = one-way A→B, 2 = two-way
        cost  = int(parts[3])   # traversal time (seconds)
        length = int(parts[4])  # length in metres
        cat   = parts[5]        # M / O / C
        req   = int(parts[6])   # cleaning requirement (L/km); 0 for connectors

        assert 0 <= a < N and 0 <= b < N and a != b, f"Street {j}: invalid junctions"
        assert d in (1, 2),                           f"Street {j}: direction must be 1 or 2"
        assert cat in CAT_VALID,                      f"Street {j}: unknown category '{cat}'"

        streets.append({
            'id': j, 'a': a, 'b': b, 'dir': d,
            'cost': cost, 'length': length,
            'cat': cat, 'req': req,
        })

    # Parse vehicle-types line: space-separated S/M/L tokens
    vehicle_tokens = raw[idx].split()
    assert len(vehicle_tokens) == C, (
        f"Expected {C} vehicle types, got {len(vehicle_tokens)}: {vehicle_tokens}")

    CAP = {'S': 10, 'M': 20, 'L': 30}
    vehicles = [{'type': t, 'cap': CAP[t]} for t in vehicle_tokens]

    return {
        'N': N, 'M': M, 'T': T, 'C': C, 'S': S, 'W': W,
        'streets': streets, 'vehicles': vehicles,
        'junction_coords': junction_coords,
    }


# ---------------------------------------------------------------------------
# Graph helpers
# ---------------------------------------------------------------------------

def build_arcs(instance: dict):
    """
    Create a directed-arc representation of the street network.

    Returns
    -------
    arcs : list of (u, v, street_id)
        Directed arcs; a two-way street j produces arcs at indices 2j and 2j+1.
    out_arcs : dict  node → list of arc indices leaving that node
    in_arcs  : dict  node → list of arc indices entering that node
    street_arcs : dict  street_id → list of arc indices for that street
    arc_cost : list[int]  traversal time for each arc (same as its street)
    """
    arcs = []
    out_arcs  = collections.defaultdict(list)
    in_arcs   = collections.defaultdict(list)
    street_arcs = collections.defaultdict(list)

    for s in instance['streets']:
        j = s['id']
        a, b = s['a'], s['b']

        # Forward arc
        fwd = len(arcs)
        arcs.append((a, b, j))
        out_arcs[a].append(fwd)
        in_arcs[b].append(fwd)
        street_arcs[j].append(fwd)

        # Backward arc (two-way streets only)
        if s['dir'] == 2:
            bwd = len(arcs)
            arcs.append((b, a, j))
            out_arcs[b].append(bwd)
            in_arcs[a].append(bwd)
            street_arcs[j].append(bwd)

    arc_cost = [instance['streets'][arcs[a][2]]['cost'] for a in range(len(arcs))]
    return arcs, out_arcs, in_arcs, street_arcs, arc_cost


def dijkstra(source: int, adj_weighted: dict, N: int):
    """
    Standard Dijkstra.  adj_weighted[u] = list of (v, cost).
    Returns dist[u] = shortest time from source to u (inf if unreachable).
    """
    dist = [float('inf')] * N
    dist[source] = 0
    heap = [(0, source)]
    while heap:
        d, u = heapq.heappop(heap)
        if d > dist[u]:
            continue
        for v, c in adj_weighted.get(u, []):
            nd = d + c
            if nd < dist[v]:
                dist[v] = nd
                heapq.heappush(heap, (nd, v))
    return dist


def dijkstra_with_prev(source: int, adj_weighted: dict, N: int):
    """Dijkstra returning (dist, prev) for path reconstruction."""
    dist = [float('inf')] * N
    prev = [-1] * N
    dist[source] = 0
    heap = [(0, source)]
    while heap:
        d, u = heapq.heappop(heap)
        if d > dist[u]:
            continue
        for v, c in adj_weighted.get(u, []):
            nd = d + c
            if nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(heap, (nd, v))
    return dist, prev


def shortest_path_nodes(source: int, target: int, adj_weighted: dict, N: int):
    """Return (cost, [node_sequence]) for shortest path, or (inf, []) if unreachable."""
    dist, prev = dijkstra_with_prev(source, adj_weighted, N)
    if dist[target] == float('inf'):
        return float('inf'), []
    path = []
    cur = target
    while cur != -1:
        path.append(cur)
        cur = prev[cur]
    path.reverse()
    return dist[target], path


# ---------------------------------------------------------------------------
# MILP model
# ---------------------------------------------------------------------------

def solve_milp(instance: dict, time_limit_sec: int, num_workers: int,
               relaxed_mandatory: set = None):
    """
    Build and solve the CP-SAT MILP.

    Parameters
    ----------
    relaxed_mandatory : set of int, optional
        Street indices that are mandatory but provably unreachable.
        These are treated as optional (soft) rather than hard constraints.

    Formulation
    -----------
    Variables
        f[a, k]       integer ≥ 0  –  number of times vehicle k traverses arc a
        clean[j, k]   binary       –  vehicle k cleans street j
        cleaned[j]    binary       –  street j is cleaned by any vehicle

    Constraints
        (1) Flow conservation at every node for each vehicle (closed walk)
        (2) Route time ≤ T for each vehicle
        (3) clean[j,k] = 0 if vehicle cannot clean street j (cap < req, or connector)
        (4) clean[j,k] ≤ traversals of street j by vehicle k
        (5) cleaned[j] ↔ ∃k: clean[j,k]
        (6) Each street cleaned by at most one vehicle  (avoids redundant waste)
        (7) All mandatory streets must be cleaned
        NOTE: route connectivity (all arcs reachable from depot) is enforced in
        post-processing via Eulerian circuit reconstruction + bridge insertion.

    Objective
        Maximise  Σ_j length[j]·cleaned[j]  −  W · Σ_{j,k} waste[j,k]·clean[j,k]
        where waste[j,k] = (cap[k] − req[j]) · length[j] / 1000  (litres)
        (scaled to integer arithmetic; scale factor 1000)
    """
    N, M, T, C, S, W = (instance[k] for k in ('N', 'M', 'T', 'C', 'S', 'W'))
    streets  = instance['streets']
    vehicles = instance['vehicles']

    arcs, out_arcs, in_arcs, street_arcs, arc_cost = build_arcs(instance)
    A = len(arcs)

    # Per-arc traversal upper bounds: floor(T / cost), capped at 50
    arc_max = [min(T // max(arc_cost[a], 1), 50) for a in range(A)]

    model = cp_model.CpModel()

    # ---- Decision variables ------------------------------------------------
    f = {}
    for k in range(C):
        for a in range(A):
            f[a, k] = model.new_int_var(0, arc_max[a], f'f_{a}_{k}')

    # clean[j, k]: BoolVar if vehicle k can clean street j, else Python int 0
    clean = {}
    for j in range(M):
        s = streets[j]
        for k in range(C):
            if s['cat'] != 'C' and vehicles[k]['cap'] >= s['req']:
                clean[j, k] = model.new_bool_var(f'clean_{j}_{k}')
            else:
                clean[j, k] = 0  # Python int – cannot clean

    # cleaned[j]: BoolVar for non-connector streets
    cleaned = {}
    for j in range(M):
        if streets[j]['cat'] != 'C':
            cleaned[j] = model.new_bool_var(f'cleaned_{j}')
        else:
            cleaned[j] = 0

    # ---- Constraints -------------------------------------------------------

    # (1) Flow conservation: for each vehicle k, at each node u,
    #     Σ outgoing arcs  ==  Σ incoming arcs
    for k in range(C):
        for u in range(N):
            out_f = [f[a, k] for a in out_arcs.get(u, [])]
            in_f  = [f[a, k] for a in in_arcs.get(u, [])]
            if out_f or in_f:
                model.add(sum(out_f) == sum(in_f))

    # (2) Time constraint
    for k in range(C):
        model.add(sum(arc_cost[a] * f[a, k] for a in range(A)) <= T)

    # (3) + (4) If vehicle k cleans street j it must have traversed it
    for j in range(M):
        s = streets[j]
        if s['cat'] == 'C':
            continue
        for k in range(C):
            if not isinstance(clean[j, k], int):
                # clean[j,k] = 1  →  at least one traversal of street j by k
                traversals = [f[a, k] for a in street_arcs[j]]
                if traversals:
                    # sum of traversals >= clean[j,k]
                    model.add(sum(traversals) >= clean[j, k])

    # (5) cleaned[j] ↔ OR_k clean[j,k]
    for j in range(M):
        if streets[j]['cat'] == 'C':
            continue
        bool_vars = [clean[j, k] for k in range(C) if isinstance(clean[j, k], cp_model.IntVar)]
        if bool_vars:
            for bv in bool_vars:
                model.add_implication(bv, cleaned[j])           # clean → cleaned
            model.add_bool_or(bool_vars + [cleaned[j].Not()])   # NOT cleaned → all 0
        else:
            model.add(cleaned[j] == 0)

    # (6) Each street cleaned by at most one vehicle
    for j in range(M):
        if streets[j]['cat'] == 'C':
            continue
        bool_vars = [clean[j, k] for k in range(C) if isinstance(clean[j, k], cp_model.IntVar)]
        if len(bool_vars) > 1:
            model.add(sum(bool_vars) <= 1)

    # (7) Mandatory streets
    if relaxed_mandatory is None:
        relaxed_mandatory = set()
    for j in range(M):
        if streets[j]['cat'] == 'M':
            if j in relaxed_mandatory:
                # Provably unreachable — treat as optional (still scored if cleaned)
                continue
            assert isinstance(cleaned[j], cp_model.IntVar), (
                f"Street {j} is mandatory but no vehicle can clean it!")
            model.add(cleaned[j] == 1)

    # (8) Depot-inclusion:  if vehicle k uses any arc at all, it must include the
    #     depot.  Big-M formulation:  total_flow_k  ≤  depot_out_k × big_M
    #     When depot_out_k = 0  →  total_flow_k ≤ 0  →  no arcs allowed.
    big_M_val = sum(arc_max) + 1
    for k in range(C):
        depot_out_arcs = out_arcs.get(S, [])
        if depot_out_arcs:
            depot_out_sum  = sum(f[a, k] for a in depot_out_arcs)
            total_flow_sum = sum(f[a, k] for a in range(A))
            model.add(total_flow_sum <= big_M_val * depot_out_sum)

    # ---- Objective ---------------------------------------------------------
    # Score = Σ_j length[j]·cleaned[j] − W·Σ_{j,k} waste[j,k]·clean[j,k]
    # Scale by SCALE=1000 to keep integers when W has decimals.
    SCALE    = 1000
    W_scaled = int(round(W * SCALE))

    obj_terms = []
    for j in range(M):
        s = streets[j]
        if s['cat'] == 'C':
            continue
        # Positive: length contribution
        if isinstance(cleaned[j], cp_model.IntVar):
            obj_terms.append(SCALE * s['length'] * cleaned[j])

        # Negative: water-waste penalty per cleaning vehicle
        for k in range(C):
            if not isinstance(clean[j, k], cp_model.IntVar):
                continue
            excess = vehicles[k]['cap'] - s['req']  # L/km over-spray
            if excess > 0 and W_scaled > 0:
                # penalty = W * excess [L/km] * length [m] / 1000 [m/km]
                #         = W_scaled/SCALE * excess * length / 1000
                # scaled  = W_scaled * excess * length / 1000
                penalty = W_scaled * excess * s['length'] // 1000
                if penalty > 0:
                    obj_terms.append(-penalty * clean[j, k])

    model.maximize(sum(obj_terms) if obj_terms else 0)

    # ---- Iterative solve with subtour elimination --------------------------
    print(f"\n[Solver] CP-SAT  |  time_limit={time_limit_sec}s  |  workers={num_workers}")
    print(f"[Model]  {A} arcs  |  {M} streets  |  {C} vehicles")

    remaining_time = float(time_limit_sec)
    max_iterations = 20
    f_vals = None
    clean_vals = None
    status_name = None

    for iteration in range(max_iterations):
        solver = cp_model.CpSolver()
        solver.parameters.max_time_in_seconds = max(remaining_time, 1.0)
        solver.parameters.num_workers         = num_workers
        solver.parameters.log_search_progress = (iteration == 0)

        t0      = time.time()
        status  = solver.solve(model)
        elapsed = time.time() - t0
        remaining_time -= elapsed

        status_name = solver.status_name(status)
        obj_val = solver.objective_value if status in (
            cp_model.OPTIMAL, cp_model.FEASIBLE) else 0
        print(f"[Iter {iteration+1}] {status_name}  |  wall={elapsed:.1f}s  "
              f"|  obj={obj_val:.0f} (scaled×{SCALE})  "
              f"|  remaining={remaining_time:.1f}s")

        if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
            if f_vals is not None:
                print("  Using best connected solution from previous iteration.")
                break
            return None, None, None, None

        # Extract current solution
        f_vals = {(a, k): solver.value(f[a, k])
                  for k in range(C) for a in range(A)}
        clean_vals = {}
        for j in range(M):
            for k in range(C):
                v = clean[j, k]
                clean_vals[j, k] = solver.value(v) if isinstance(
                    v, cp_model.IntVar) else 0

        # --- Check for subtours (disconnected components w/o depot) ----------
        subtours_found = False
        for k in range(C):
            total_used = sum(f_vals[a, k] for a in range(A))
            if total_used == 0:
                continue
            adj_k = _build_adj_multigraph(k, f_vals, arcs)
            comps = _connected_components(adj_k)
            for comp in comps:
                if S in comp:
                    continue
                # Found a subtour disconnected from the depot
                internal = [a for a in range(A)
                            if arcs[a][0] in comp and arcs[a][1] in comp]
                entering = [a for a in range(A)
                            if arcs[a][0] not in comp and arcs[a][1] in comp]
                if not internal:
                    continue
                if entering:
                    big_M_sub = sum(arc_max[a] for a in internal) + 1
                    model.add(
                        sum(f[a, k] for a in internal)
                        <= big_M_sub * sum(f[a, k] for a in entering)
                    )
                else:
                    # No entering arcs exist → impossible to connect
                    for a in internal:
                        model.add(f[a, k] == 0)
                subtours_found = True
                print(f"  [Subtour] Vehicle {k}: "
                      f"eliminated component {comp}")

        if not subtours_found:
            print(f"  No subtours – all vehicle flows are connected.")
            break

        if remaining_time <= 2.0:
            print(f"  [Warning] Time budget exhausted during "
                  f"subtour elimination.")
            break

        # Hint solver with current solution for faster re-solve
        if hasattr(model, 'clear_hints'):
            model.clear_hints()
        for k in range(C):
            for a in range(A):
                model.add_hint(f[a, k], f_vals[a, k])
            for j in range(M):
                if isinstance(clean[j, k], cp_model.IntVar):
                    model.add_hint(clean[j, k], clean_vals[j, k])

    return f_vals, clean_vals, (arcs, out_arcs, in_arcs, street_arcs, arc_cost), status_name


# ---------------------------------------------------------------------------
# Route reconstruction
# ---------------------------------------------------------------------------

def _hierholzer(adj: collections.defaultdict, start: int) -> list:
    """
    Hierholzer's algorithm on a defaultdict-based multigraph.
    Returns the Eulerian circuit as a node list (starts and ends at *start*).
    Modifies adj in-place (edge lists are consumed).
    """
    stack   = [start]
    circuit = []
    while stack:
        u = stack[-1]
        if adj[u]:          # defaultdict: safe, returns [] for missing keys
            v = adj[u].pop()[0]
            stack.append(v)
        else:
            circuit.append(stack.pop())
    circuit.reverse()
    return circuit


def _build_adj_multigraph(k: int, f_vals: dict, arcs: list) -> dict:
    """Build adjacency list (as list of [v]) for vehicle k from f_vals."""
    adj = collections.defaultdict(list)
    for a_idx, (u, v, _j) in enumerate(arcs):
        cnt = f_vals[a_idx, k]
        for _ in range(cnt):
            adj[u].append([v])
    return adj


def _connected_components(adj: dict) -> list:
    """
    Return list of sets of nodes, each set being one weakly-connected component
    of the multigraph (ignoring arc direction for connectivity analysis).
    Uses undirected adjacency built from the directed adj.
    """
    all_nodes = set(adj.keys())
    # add destination nodes
    for u in list(all_nodes):
        for edge in adj[u]:
            all_nodes.add(edge[0])

    visited = set()
    components = []
    for start in all_nodes:
        if start in visited:
            continue
        # BFS (undirected)
        comp = set()
        queue = collections.deque([start])
        comp.add(start)
        while queue:
            u = queue.popleft()
            visited.add(u)
            for edge in adj.get(u, []):
                v = edge[0]
                if v not in comp:
                    comp.add(v)
                    queue.append(v)
        components.append(comp)
    return components


def reconstruct_route(k: int, f_vals: dict, arcs: list, instance: dict,
                      adj_weighted: dict) -> tuple:
    """
    Build the actual junction-sequence route for vehicle k.

    Steps
    -----
    1. Build multigraph G_k from f_vals.
    2. Detect connected components; if any component does not include the depot,
       attempt to bridge it (depot → comp → depot) within the remaining time budget.
       If bridging is impossible, remove those arcs from the multigraph (they cannot
       be part of a valid route) and record the dropped street indices.
    3. Run Hierholzer's algorithm on the repaired multigraph.
    4. Return (route_node_list, dropped_streets_set).
    """
    depot = instance['S']
    T     = instance['T']

    adj        = _build_adj_multigraph(k, f_vals, arcs)
    total_used = sum(f_vals[a_idx, k] for a_idx in range(len(arcs)))

    if total_used == 0:
        return [depot], set()

    # Current route time (used for budget checks during connectivity repair)
    current_time = sum(
        instance['streets'][arcs[a][2]]['cost'] * f_vals[a, k]
        for a in range(len(arcs))
    )

    dropped_streets: set = set()

    # ---- Connectivity repair -----------------------------------------------
    comps = _connected_components(adj)

    for comp in comps:
        if depot in comp:
            continue  # already connected to depot

        # Find cheapest round-trip bridge: depot → best_node_in_comp → depot
        best_cost = float('inf')
        best_path: list = []
        for node in comp:
            c, path = shortest_path_nodes(depot, node, adj_weighted, instance['N'])
            if c < best_cost:
                best_cost = c
                best_path = path

        if best_path and 2 * best_cost + current_time <= T:
            # Add forward path: depot → comp_node
            for i in range(len(best_path) - 1):
                adj[best_path[i]].append([best_path[i + 1]])
            # Add backward path: comp_node → depot
            rev = list(reversed(best_path))
            for i in range(len(rev) - 1):
                adj[rev[i]].append([rev[i + 1]])
            current_time += 2 * best_cost
            print(f"  [Repair] Vehicle {k}: bridged isolated component "
                  f"(+{2 * best_cost}s, running total {current_time}s)")
        else:
            # Cannot bridge → remove these arcs so Hierholzer stays valid
            if not best_path:
                print(f"  [Warning] Vehicle {k}: component {comp} unreachable "
                      f"from depot; dropping its arcs.")
            else:
                print(f"  [Warning] Vehicle {k}: bridge cost {2 * best_cost}s "
                      f"would exceed time budget; dropping component {comp}.")
            # Record which streets are being dropped
            for node in comp:
                adj[node].clear()
            # Mark streets whose arcs were only in this component as dropped
            for a_idx, (u, v, j) in enumerate(arcs):
                if u in comp and f_vals[a_idx, k] > 0:
                    dropped_streets.add(j)

    # ---- Guard: if depot has no outgoing arcs, nothing to route -------------
    if not adj[depot]:
        return [depot], dropped_streets

    # ---- Eulerian circuit (Hierholzer on defaultdict adj) -------------------
    circuit = _hierholzer(adj, depot)

    if not circuit:
        return [depot], dropped_streets
    if circuit[-1] != depot:
        circuit.append(depot)

    return circuit, dropped_streets


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------

def validate_solution(instance: dict, routes: list, clean_per_vehicle: list) -> tuple:
    """
    Validate the solution against all problem rules.

    Returns (is_valid: bool, errors: list[str]).
    """
    streets  = instance['streets']
    vehicles = instance['vehicles']
    T, S, M, C = instance['T'], instance['S'], instance['M'], instance['C']

    # Build arc lookup: (u,v) → street_id (for any valid directed traversal)
    arc_to_street = {}
    for s in streets:
        arc_to_street[(s['a'], s['b'])] = s['id']
        if s['dir'] == 2:
            arc_to_street[(s['b'], s['a'])] = s['id']

    errors      = []
    all_cleaned = set()

    for k in range(C):
        route   = routes[k]
        cleaned = set(clean_per_vehicle[k])

        # Route must start and end at depot
        if route[0] != S:
            errors.append(f"Vehicle {k}: route starts at {route[0]}, not depot {S}")
        if route[-1] != S:
            errors.append(f"Vehicle {k}: route ends at {route[-1]}, not depot {S}")

        # Traverse route, accumulate time, find traversed streets
        total_time      = 0
        traversed_set   = set()
        valid_traverse  = True

        for i in range(len(route) - 1):
            u, v = route[i], route[i + 1]
            sid  = arc_to_street.get((u, v))
            if sid is None:
                errors.append(f"Vehicle {k}: no arc ({u} → {v}) at step {i}")
                valid_traverse = False
                break
            traversed_set.add(sid)
            total_time += streets[sid]['cost']

        if total_time > T:
            errors.append(f"Vehicle {k}: route time {total_time}s exceeds limit {T}s")

        # Cleaning checks
        for j in cleaned:
            if j < 0 or j >= M:
                errors.append(f"Vehicle {k}: invalid street index {j}")
                continue
            s = streets[j]
            if valid_traverse and j not in traversed_set:
                errors.append(f"Vehicle {k}: cleans street {j} but never traverses it")
            if vehicles[k]['cap'] < s['req']:
                errors.append(
                    f"Vehicle {k} (cap={vehicles[k]['cap']}): cannot clean street {j} "
                    f"(req={s['req']})")
            if s['cat'] == 'C':
                errors.append(f"Vehicle {k}: cannot clean connector street {j}")

        all_cleaned.update(cleaned)

    # All mandatory streets must appear in all_cleaned
    for j in range(M):
        if streets[j]['cat'] == 'M' and j not in all_cleaned:
            errors.append(f"Mandatory street {j} not cleaned by any vehicle!")

    return len(errors) == 0, errors


# ---------------------------------------------------------------------------
# Scoring
# ---------------------------------------------------------------------------

def compute_score(instance: dict, clean_per_vehicle: list) -> tuple:
    """
    Compute and return (score, total_length_m, total_waste_L, cleaned_set).
    """
    streets  = instance['streets']
    vehicles = instance['vehicles']
    W        = instance['W']

    cleaned_length  = {}   # j → length (accumulated once)
    total_waste     = 0.0

    for k, cleaned in enumerate(clean_per_vehicle):
        for j in cleaned:
            s = streets[j]
            if s['cat'] == 'C':
                continue
            waste = (vehicles[k]['cap'] - s['req']) * s['length'] / 1000.0
            total_waste += waste
            if j not in cleaned_length:
                cleaned_length[j] = s['length']

    total_length = sum(cleaned_length.values())
    score        = total_length - W * total_waste
    return score, total_length, total_waste, set(cleaned_length.keys())


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

def write_output(filepath: str, routes: list, clean_per_vehicle: list, C: int):
    """
    Write solution in the required CLIPP submission format.

    Format per vehicle (3 lines):
        n                         <- number of streets traversed  (= len(route)-1)
        j_0 j_1 ... j_n          <- n+1 junction indices (starts and ends at depot)
        s_0 s_1 ...              <- street indices cleaned (empty line if none)

    Edge case: if the vehicle never leaves the depot (route=[S]) write n=1, line="S".
    """
    with open(filepath, 'w') as fh:
        fh.write(f"{C}\n")
        for k in range(C):
            route   = routes[k]
            cleaned = sorted(clean_per_vehicle[k])
            # n = number of streets traversed = len(route) - 1;
            # minimum n=1 for depot-only (write single junction)
            n = max(len(route) - 1, 1)
            fh.write(f"{n}\n")
            fh.write(" ".join(map(str, route)) + "\n")
            fh.write((" ".join(map(str, cleaned))) + "\n")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='CLIPP MILP Solver (OR-Tools CP-SAT)',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument('input_file', help='Path to instance .txt file')
    parser.add_argument('--time_limit',  type=int,   default=600,
                        help='Solver time limit in seconds')
    parser.add_argument('--num_workers', type=int,   default=4,
                        help='Number of parallel solver workers (CPU cores)')
    parser.add_argument('--output',      type=str,   default=None,
                        help='Output file path (default: <input_stem>_solution.txt)')
    args = parser.parse_args()

    # Determine output path
    if args.output:
        output_file = args.output
    else:
        p = Path(args.input_file)
        output_file = str(p.parent / (p.stem + '_solution.txt'))

    print("=" * 60)
    print("CLIPP MILP Solver")
    print("=" * 60)
    print(f"Input   : {args.input_file}")
    print(f"Output  : {output_file}")
    print(f"Limit   : {args.time_limit}s  |  Workers: {args.num_workers}")

    wall_start = time.time()

    # --- Parse ---
    instance = parse_instance(args.input_file)
    N, M, T, C, S, W = (instance[k] for k in ('N', 'M', 'T', 'C', 'S', 'W'))
    cats = collections.Counter(s['cat'] for s in instance['streets'])
    print(f"\nInstance: N={N}  M={M}  T={T}s  C={C}  depot={S}  W={W}")
    print(f"  Streets: {cats['M']} mandatory  {cats['O']} optional  {cats['C']} connector")
    print(f"  Fleet  : {[v['type'] for v in instance['vehicles']]}")

    # --- Check feasibility: can we reach / return from all mandatory streets? ---
    arcs, out_arcs_map, in_arcs_map, street_arcs_map, arc_cost_list = \
        build_arcs(instance)

    # Build weighted adjacency for Dijkstra (used later for connectivity repair)
    adj_weighted = collections.defaultdict(list)
    for (u, v, j) in arcs:
        adj_weighted[u].append((v, instance['streets'][j]['cost']))

    # Quick reachability check from depot
    dist_from_depot = dijkstra(S, adj_weighted, N)

    # Build reverse adjacency for distance-to-depot computation
    rev_adj = collections.defaultdict(list)
    for u, neighbours in adj_weighted.items():
        for v, c in neighbours:
            rev_adj[v].append((u, c))
    dist_to_depot = dijkstra(S, rev_adj, N)

    unreachable_mandatory = []
    for s in instance['streets']:
        if s['cat'] == 'M':
            a, b = s['a'], s['b']
            # Need to reach a (from depot) and return from b (or vice versa for two-way)
            reachable = (dist_from_depot[a] + instance['streets'][s['id']]['cost']
                         + dist_to_depot[b] <= T)
            if s['dir'] == 2:
                reachable = reachable or (
                    dist_from_depot[b] + instance['streets'][s['id']]['cost']
                    + dist_to_depot[a] <= T)
            if not reachable:
                unreachable_mandatory.append(s['id'])

    if unreachable_mandatory:
        print(f"\n[ERROR] Mandatory streets unreachable within time limit: "
              f"{unreachable_mandatory}")
        print("  The problem may be infeasible!")
    else:
        print(f"\n  All mandatory streets appear reachable within T={T}s.")

    # --- Solve ---
    f_vals, clean_vals, graph_info, status_name = solve_milp(
        instance, args.time_limit, args.num_workers,
        relaxed_mandatory=set(unreachable_mandatory))

    if f_vals is None:
        print("\n[FAIL] No feasible solution found. Exiting.")
        sys.exit(1)

    arcs_list, _, _, _, _ = graph_info

    # --- Reconstruct routes ---
    print("\n[Reconstruction] Building vehicle routes from arc flows...")
    routes           = []
    clean_per_vehicle = []

    for k in range(C):
        route, dropped = reconstruct_route(
            k, f_vals, arcs_list, instance, adj_weighted)
        routes.append(route)

        cleaned = [j for j in range(M)
                   if clean_vals.get((j, k), 0) and j not in dropped]
        clean_per_vehicle.append(cleaned)

        route_time = sum(
            instance['streets'][arcs_list[a][2]]['cost'] * f_vals[a, k]
            for a in range(len(arcs_list))
        )
        drop_note = f", dropped {len(dropped)} streets" if dropped else ""
        print(f"  Vehicle {k} ({instance['vehicles'][k]['type']}): "
              f"{len(route)} junctions, {len(cleaned)} streets cleaned, "
              f"route time={route_time}s{drop_note}")

    # --- Validate ---
    print("\n[Validation]")
    valid, errors = validate_solution(instance, routes, clean_per_vehicle)
    if valid:
        print("  Solution is VALID. ✓")
    else:
        print(f"  Solution has {len(errors)} error(s):")
        for e in errors:
            print(f"    - {e}")

    # --- Score ---
    score, total_length, total_waste, cleaned_set = compute_score(
        instance, clean_per_vehicle)

    mandatory_cleaned = sum(
        1 for j in cleaned_set if instance['streets'][j]['cat'] == 'M')
    optional_cleaned  = sum(
        1 for j in cleaned_set if instance['streets'][j]['cat'] == 'O')

    print(f"\n[Score]")
    print(f"  Total length cleaned : {total_length} m")
    print(f"  Total water wasted   : {total_waste:.3f} L")
    print(f"  W penalty factor     : {W}")
    print(f"  Score                : {score:.2f}")
    print(f"  Mandatory cleaned    : {mandatory_cleaned}/{cats['M']}")
    print(f"  Optional cleaned     : {optional_cleaned}/{cats['O']}")
    print(f"  Solver status        : {status_name}")

    # --- Write output ---
    write_output(output_file, routes, clean_per_vehicle, C)
    print(f"\n[Output] Written to {output_file}")
    print(f"[Time]   Total wall time: {time.time() - wall_start:.1f}s")
    print("=" * 60)


if __name__ == '__main__':
    main()
