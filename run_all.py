#!/usr/bin/env python3
"""
Batch runner for all CLIPP toy instances.

Usage:
    python run_all.py [--time_limit 120] [--num_workers 4]
"""

import argparse
import subprocess
import sys
import time
from pathlib import Path

INSTANCES = [
    "1. Instances/toy.txt",
    "1. Instances/toy_version_2.txt",
    "1. Instances/toy_version_3.txt",
    "1. Instances/toy_version_4.txt",
    "1. Instances/toy_version_5.txt",
    "1. Instances/toy_version_6.txt",
]


def run_instance(instance_path: str, time_limit: int, num_workers: int) -> dict:
    """Run clipp_solver.py on one instance, return parsed results."""
    cmd = [
        sys.executable, "clipp_solver.py", instance_path,
        "--time_limit", str(time_limit),
        "--num_workers", str(num_workers),
    ]
    t0 = time.time()
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=time_limit + 60)
    elapsed = time.time() - t0

    stdout = result.stdout + result.stderr
    info = {
        "instance": Path(instance_path).stem,
        "exit_code": result.returncode,
        "wall_time": elapsed,
        "status": "UNKNOWN",
        "valid": False,
        "score": 0.0,
        "mandatory": "?/?",
        "optional": "?/?",
        "length_cleaned": 0,
        "water_wasted": 0.0,
    }

    for line in stdout.splitlines():
        line = line.strip()
        if "INFEASIBLE" in line and "Result" in line:
            info["status"] = "INFEASIBLE"
        elif "OPTIMAL" in line and ("Iter" in line or "Result" in line):
            info["status"] = "OPTIMAL"
        elif "FEASIBLE" in line and ("Iter" in line or "Result" in line):
            info["status"] = "FEASIBLE"
        elif "Solution is VALID" in line:
            info["valid"] = True
        elif "Score" in line and ":" in line and "score" not in line.lower():
            pass
        elif line.startswith("Score"):
            try:
                info["score"] = float(line.split(":")[-1].strip())
            except ValueError:
                pass
        elif "Total length cleaned" in line:
            try:
                info["length_cleaned"] = int(line.split(":")[-1].strip().split()[0])
            except (ValueError, IndexError):
                pass
        elif "Total water wasted" in line:
            try:
                info["water_wasted"] = float(line.split(":")[-1].strip().split()[0])
            except (ValueError, IndexError):
                pass
        elif "Mandatory cleaned" in line:
            info["mandatory"] = line.split(":")[-1].strip()
        elif "Optional cleaned" in line:
            info["optional"] = line.split(":")[-1].strip()
        elif "FAIL" in line:
            info["status"] = "FAIL"

    return info


def main():
    parser = argparse.ArgumentParser(description="Run CLIPP solver on all toy instances")
    parser.add_argument("--time_limit", type=int, default=120,
                        help="Solver time limit per instance (seconds)")
    parser.add_argument("--num_workers", type=int, default=4,
                        help="Number of parallel solver workers")
    args = parser.parse_args()

    print("=" * 80)
    print("CLIPP Batch Runner — All Toy Instances")
    print(f"Time limit: {args.time_limit}s per instance  |  Workers: {args.num_workers}")
    print("=" * 80)

    results = []
    total_start = time.time()

    for inst_path in INSTANCES:
        if not Path(inst_path).exists():
            print(f"\n[SKIP] {inst_path} not found.")
            continue

        name = Path(inst_path).stem
        print(f"\n{'─' * 60}")
        print(f"Running: {name}")
        print(f"{'─' * 60}")

        try:
            info = run_instance(inst_path, args.time_limit, args.num_workers)
        except subprocess.TimeoutExpired:
            info = {
                "instance": name,
                "exit_code": -1,
                "wall_time": args.time_limit,
                "status": "TIMEOUT",
                "valid": False,
                "score": 0.0,
                "mandatory": "?/?",
                "optional": "?/?",
                "length_cleaned": 0,
                "water_wasted": 0.0,
            }

        valid_str = "VALID" if info["valid"] else "INVALID"
        print(f"  Status: {info['status']}  |  {valid_str}  |  "
              f"Score: {info['score']:.2f}  |  Time: {info['wall_time']:.1f}s")
        print(f"  Mandatory: {info['mandatory']}  |  Optional: {info['optional']}")
        results.append(info)

    total_time = time.time() - total_start

    # Summary table
    print(f"\n{'=' * 80}")
    print("SUMMARY")
    print(f"{'=' * 80}")
    print(f"{'Instance':<22} {'Status':<12} {'Valid':<7} {'Score':>10} "
          f"{'Mandatory':>10} {'Optional':>10} {'Time':>8}")
    print("-" * 80)

    total_score = 0.0
    for r in results:
        valid_str = "YES" if r["valid"] else "NO"
        print(f"{r['instance']:<22} {r['status']:<12} {valid_str:<7} "
              f"{r['score']:>10.2f} {r['mandatory']:>10} {r['optional']:>10} "
              f"{r['wall_time']:>7.1f}s")
        if r["valid"]:
            total_score += r["score"]

    print("-" * 80)
    print(f"{'TOTAL (valid only)':<22} {'':12} {'':7} {total_score:>10.2f}")
    print(f"\nTotal wall time: {total_time:.1f}s")
    print("=" * 80)


if __name__ == "__main__":
    main()
