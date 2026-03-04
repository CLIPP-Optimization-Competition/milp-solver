#!/usr/bin/env python3
"""
Check CLIPP feasibility.

Modes
-----
1) Instance screening (no solution file):
   - Proves some cases are INFEASIBLE (e.g., impossible mandatory streets).
   - Otherwise reports UNKNOWN (not a full MILP feasibility proof).

2) Solution validation (with --solution):
   - Returns FEASIBLE or INFEASIBLE for the provided solution file.

Usage
-----
python3 check_feasibility.py --instance Instances/toy_version_2.txt
python3 check_feasibility.py --instance Instances/toy.txt --solution Instances/toy_solution.txt
"""

import argparse
import collections
import heapq
from pathlib import Path


def parse_instance(filepath):
    with open(filepath, encoding="utf-8") as fh:
        raw = [ln.strip() for ln in fh if ln.strip()]

    idx = 0
    header = raw[idx].split()
    idx += 1

    n = int(header[0])
    m = int(header[1])
    t_limit = int(header[2])
    c = int(header[3])
    depot = int(header[4])
    w_penalty = float(header[5]) if len(header) >= 6 else 0.0

    # Optional coordinate block
    junction_coords = []
    if idx < len(raw):
        tokens = raw[idx].split()
        if len(tokens) == 2:
            try:
                float(tokens[0])
                float(tokens[1])
                for _ in range(n):
                    parts = raw[idx].split()
                    junction_coords.append((float(parts[0]), float(parts[1])))
                    idx += 1
            except ValueError:
                pass

    streets = []
    for j in range(m):
        parts = raw[idx].split()
        idx += 1
        streets.append(
            {
                "id": j,
                "a": int(parts[0]),
                "b": int(parts[1]),
                "dir": int(parts[2]),
                "cost": int(parts[3]),
                "length": int(parts[4]),
                "cat": parts[5],
                "req": int(parts[6]),
            }
        )

    tokens = raw[idx].split()
    cap_map = {"S": 10, "M": 20, "L": 30}
    vehicles = [{"type": tok, "cap": cap_map[tok]} for tok in tokens]

    return {
        "N": n,
        "M": m,
        "T": t_limit,
        "C": c,
        "S": depot,
        "W": w_penalty,
        "streets": streets,
        "vehicles": vehicles,
        "junction_coords": junction_coords,
    }


def parse_solution(filepath):
    lines = Path(filepath).read_text(encoding="utf-8").splitlines()
    if not lines:
        raise ValueError("Solution file is empty.")

    idx = 0
    c_sol = int(lines[idx].strip())
    idx += 1

    routes = []
    clean_per_vehicle = []

    for k in range(c_sol):
        if idx >= len(lines):
            raise ValueError(f"Missing block for vehicle {k}.")
        n = int(lines[idx].strip())
        idx += 1

        if idx >= len(lines):
            raise ValueError(f"Missing route line for vehicle {k}.")
        route_line = lines[idx].strip()
        idx += 1
        route = [int(x) for x in route_line.split()] if route_line else []

        if idx >= len(lines):
            cleaned_line = ""
        else:
            cleaned_line = lines[idx].strip()
            idx += 1
        cleaned = [int(x) for x in cleaned_line.split()] if cleaned_line else []

        routes.append(route)
        clean_per_vehicle.append(cleaned)

        # n is expected to be traversed street count
        expected_n = max(len(route) - 1, 1)
        if n != expected_n:
            # Keep parsing; this alone does not invalidate feasibility.
            pass

    return c_sol, routes, clean_per_vehicle


def build_directed_adjacency(instance):
    adj = collections.defaultdict(list)
    rev = collections.defaultdict(list)
    for s in instance["streets"]:
        a, b, cost = s["a"], s["b"], s["cost"]
        adj[a].append((b, cost))
        rev[b].append((a, cost))
        if s["dir"] == 2:
            adj[b].append((a, cost))
            rev[a].append((b, cost))
    return adj, rev


def dijkstra(source, adj, n):
    dist = [float("inf")] * n
    dist[source] = 0
    heap = [(0, source)]

    while heap:
        d, u = heapq.heappop(heap)
        if d > dist[u]:
            continue
        for v, c in adj.get(u, []):
            nd = d + c
            if nd < dist[v]:
                dist[v] = nd
                heapq.heappush(heap, (nd, v))
    return dist


def screen_instance_feasibility(instance):
    """
    Returns:
        status: "INFEASIBLE" or "UNKNOWN"
        reasons: list[str]
    """
    n, t_limit, depot = instance["N"], instance["T"], instance["S"]
    vehicles = instance["vehicles"]
    streets = instance["streets"]

    adj, rev = build_directed_adjacency(instance)
    dist_from = dijkstra(depot, adj, n)
    dist_to = dijkstra(depot, rev, n)

    reasons = []

    for s in streets:
        if s["cat"] != "M":
            continue

        capable = [k for k, v in enumerate(vehicles) if v["cap"] >= s["req"]]
        if not capable:
            reasons.append(
                f"Mandatory street {s['id']} needs req={s['req']} but no vehicle has enough capacity."
            )
            continue

        best = float("inf")
        if s["dir"] == 1:
            a_to_b = dist_from[s["a"]] + s["cost"] + dist_to[s["b"]]
            best = min(best, a_to_b)
        else:
            a_to_b = dist_from[s["a"]] + s["cost"] + dist_to[s["b"]]
            b_to_a = dist_from[s["b"]] + s["cost"] + dist_to[s["a"]]
            best = min(best, a_to_b, b_to_a)

        if best == float("inf"):
            reasons.append(
                f"Mandatory street {s['id']} is unreachable in directed graph from/to depot {depot}."
            )
        elif best > t_limit:
            reasons.append(
                f"Mandatory street {s['id']} requires at least {int(best)}s for depot->street->depot, "
                f"exceeding T={t_limit}s."
            )

    if reasons:
        return "INFEASIBLE", reasons
    return "UNKNOWN", ["No immediate mandatory-street contradiction found."]


def validate_solution(instance, routes, clean_per_vehicle):
    streets = instance["streets"]
    vehicles = instance["vehicles"]
    t_limit = instance["T"]
    depot = instance["S"]
    m = instance["M"]
    c = instance["C"]

    errors = []

    if len(routes) != c or len(clean_per_vehicle) != c:
        errors.append(
            f"Solution has {len(routes)} vehicle blocks but instance expects C={c}."
        )
        return False, errors

    arc_to_street = {}
    for s in streets:
        arc_to_street[(s["a"], s["b"])] = s["id"]
        if s["dir"] == 2:
            arc_to_street[(s["b"], s["a"])] = s["id"]

    global_clean_owner = {}

    for k in range(c):
        route = routes[k]
        cleaned = clean_per_vehicle[k]

        if not route:
            errors.append(f"Vehicle {k}: empty route.")
            continue

        if route[0] != depot:
            errors.append(f"Vehicle {k}: route must start at depot {depot}, got {route[0]}.")
        if route[-1] != depot:
            errors.append(f"Vehicle {k}: route must end at depot {depot}, got {route[-1]}.")

        traversed = set()
        route_time = 0
        valid_traversal = True

        for i in range(len(route) - 1):
            u, v = route[i], route[i + 1]
            sid = arc_to_street.get((u, v))
            if sid is None:
                errors.append(f"Vehicle {k}: invalid traversal {u}->{v} at step {i}.")
                valid_traversal = False
                break
            traversed.add(sid)
            route_time += streets[sid]["cost"]

        if route_time > t_limit:
            errors.append(
                f"Vehicle {k}: route time {route_time}s exceeds T={t_limit}s."
            )

        seen_local = set()
        for sid in cleaned:
            if sid in seen_local:
                errors.append(f"Vehicle {k}: street {sid} listed multiple times in cleaned set.")
                continue
            seen_local.add(sid)

            if sid < 0 or sid >= m:
                errors.append(f"Vehicle {k}: invalid street index {sid} in cleaned set.")
                continue

            s = streets[sid]
            if s["cat"] == "C":
                errors.append(f"Vehicle {k}: connector street {sid} cannot be cleaned.")
            if vehicles[k]["cap"] < s["req"]:
                errors.append(
                    f"Vehicle {k}: cap={vehicles[k]['cap']} cannot clean street {sid} req={s['req']}."
                )
            if valid_traversal and sid not in traversed:
                errors.append(
                    f"Vehicle {k}: street {sid} marked cleaned but never traversed."
                )

            if sid in global_clean_owner and global_clean_owner[sid] != k:
                prev = global_clean_owner[sid]
                errors.append(
                    f"Street {sid} is cleaned by multiple vehicles ({prev} and {k})."
                )
            else:
                global_clean_owner[sid] = k

    mandatory = [s["id"] for s in streets if s["cat"] == "M"]
    cleaned_all = set(global_clean_owner.keys())
    for sid in mandatory:
        if sid not in cleaned_all:
            errors.append(f"Mandatory street {sid} is not cleaned.")

    return len(errors) == 0, errors


def main():
    parser = argparse.ArgumentParser(
        description="Check CLIPP instance/solution feasibility."
    )
    parser.add_argument(
        "--instance",
        required=True,
        help="Path to CLIPP instance file (.txt).",
    )
    parser.add_argument(
        "--solution",
        default=None,
        help="Path to solution file. If omitted, only instance screening is run.",
    )
    args = parser.parse_args()

    instance = parse_instance(args.instance)

    if args.solution is None:
        status, reasons = screen_instance_feasibility(instance)
        print(f"INSTANCE: {Path(args.instance)}")
        print(f"STATUS  : {status}")
        for r in reasons:
            print(f"- {r}")
        return

    c_sol, routes, clean_per_vehicle = parse_solution(args.solution)
    if c_sol != instance["C"]:
        print(f"INSTANCE: {Path(args.instance)}")
        print(f"SOLUTION: {Path(args.solution)}")
        print("STATUS  : INFEASIBLE")
        print(
            f"- Vehicle-count mismatch: solution has {c_sol}, instance expects {instance['C']}."
        )
        return

    ok, errors = validate_solution(instance, routes, clean_per_vehicle)
    print(f"INSTANCE: {Path(args.instance)}")
    print(f"SOLUTION: {Path(args.solution)}")
    print(f"STATUS  : {'FEASIBLE' if ok else 'INFEASIBLE'}")
    if errors:
        for e in errors:
            print(f"- {e}")
    else:
        print("- All validation checks passed.")


if __name__ == "__main__":
    main()
