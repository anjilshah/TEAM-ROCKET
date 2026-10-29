import argparse
import csv
from statistics import mean

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="?", default="results/evaluation/trials.csv")
    args = parser.parse_args()
    with open(args.csv) as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        parser.error("CSV has no trials; run evaluate_success.py first")
    for field in ("simulation_seconds", "wall_seconds", "average_simulation_seconds_per_peg"):
        values = [float(row[field]) for row in rows]
        print(f"{field}: mean={mean(values):.3f}, min={min(values):.3f}, max={max(values):.3f}")
