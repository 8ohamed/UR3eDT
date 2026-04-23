"""Compute DT model errors for every PT scenario and write results to CSV.

Reads the full PT scenario file, groups rows by ``scenario_id``, calls
``compute_scenario_error`` for each group, and writes a flat CSV where every
row is one time-step of one scenario.

Output columns
--------------
scenario_id,
start_0 … start_5,: joint start positions (rad)
target_0 … target_5,: joint target positions (rad)
max_velocity, commanded max velocity  (deg/s)
acceleration, commanded acceleration  (deg/s²)
step, 0-based step index within the scenario
error_joint_0 … error_joint_5 PT observed  DT predicted  (rad)

Usage
-----
    python anomaly_detection/training/compute_model_error.py

The output file is written to:
    anomaly_detection/training/data/model_errors.csv
"""

import csv
import os
import sys

import numpy as np
import pandas as pd

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, os.pardir))
sys.path.insert(0, ROOT)

from anomaly_detection.error import compute_scenario_error, load_params

INPUT_FILE = os.path.join(
    ROOT, "anomaly_detection", "training", "data", "scenarios.csv"
)
OUTPUT_DIR = os.path.join(ROOT, "anomaly_detection", "training", "data")
OUTPUT_FILE = os.path.join(OUTPUT_DIR, "model_errors.csv")

NUM_JOINTS = 6

# CSV header
HEADER = (
    ["scenario_id"]
    + [f"start_{i}" for i in range(NUM_JOINTS)]
    + [f"target_{i}" for i in range(NUM_JOINTS)]
    + ["max_velocity", "acceleration", "step"]
    + [f"error_joint_{i}" for i in range(NUM_JOINTS)]
)


def main() -> None:
    print(f"Reading scenarios from:\n  {INPUT_FILE}\n")
    df = pd.read_csv(INPUT_FILE)

    params = load_params()
    scenario_ids = df["scenario_id"].unique()
    total = len(scenario_ids)
    print(f"Found {total} unique scenarios.\n")

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    with open(OUTPUT_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(HEADER)

        for idx, sid in enumerate(scenario_ids):
            scenario_df = df[df["scenario_id"] == sid]

            header_row, errors = compute_scenario_error(scenario_df, params)

            # header_row: [start_0..5, target_0..5, max_velocity, acceleration]
            for step, error_row in enumerate(errors):
                writer.writerow(
                    [sid]
                    + list(header_row)          # 14 values: start, target, vel, acc
                    + [step]
                    + list(error_row)           # 6 values: error per joint
                )

            if (idx + 1) % 100 == 0 or idx == 0:
                print(f"  [{idx + 1:>4}/{total}] scenario {sid} – {len(errors)} steps")
                f.flush()

    print(f"\nDone errors saved to:\n  {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
