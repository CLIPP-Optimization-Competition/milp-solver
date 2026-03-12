#!/usr/bin/env python3
"""
CLIPP MILP Solver using OR-Tools CP-SAT
========================================
Solves the Climate-Integrated Street Cleaning Routing Problem (CLIPP).
Imports solver logic from the clipp/ package modules.

Usage:
    python clipp_solver.py <input_file> [--time_limit <seconds>] [--num_workers <cores>] [--output <file>]
"""

import argparse
import collections
import sys
import time
from pathlib import Path

from clipp.parser import parse_instance
from clipp.graph import build_arcs, dijkstra
from clipp.model import solve_milp
from clipp.reconstruction import reconstruct_route
from clipp.validator import validate_solution
from clipp.scoring import compute_score
from clipp.io import write_output


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

    # --- Build arcs & weighted adjacency ---
    arcs, out_arcs_map, in_arcs_map, street_arcs_map, arc_cost_list = \
        build_arcs(instance)

    adj_weighted = collections.defaultdict(list)
    for (u, v, j) in arcs:
        adj_weighted[u].append((v, instance['streets'][j]['cost']))

    # --- Reachability check ---
    dist_from_depot = dijkstra(S, adj_weighted, N)

    rev_adj = collections.defaultdict(list)
    for u, neighbours in adj_weighted.items():
        for v, c in neighbours:
            rev_adj[v].append((u, c))
    dist_to_depot = dijkstra(S, rev_adj, N)

    unreachable_mandatory = []
    for s in instance['streets']:
        if s['cat'] == 'M':
            a, b = s['a'], s['b']
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
