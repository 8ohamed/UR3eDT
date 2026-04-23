"""Collect PT (physical twin) training data by executing structured scenarios
on the UR3e mockup via RabbitMQ.

7 batches × 12 targets × 6 velocities × 6 accelerations = 3024 scenarios.
Before each scenario the robot is reset to the batch's starting position
(not recorded), then the actual scenario runs and its data is logged.

Prerequisites: UR3e mockup and RabbitMQ must be running.
"""

import sys
import os
import csv
import time
import itertools
import numpy as np
from pathlib import Path

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, os.pardir, os.pardir))
sys.path.insert(0, ROOT)

from communication import protocol
from communication.rabbitmq import Rabbitmq

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
NUM_JOINTS = 6

# Batch starting positions (radians) – all 6 joints move to this value before
# each scenario within the batch.
BATCH_START_POSITIONS = [
    0.0001,    # batch 1
    -1.309,    # batch 2
    -0.785398, # batch 3
    -0.261799, # batch 4
     0.261799, # batch 5
     0.785398, # batch 6
     1.309,    # batch 7
]

# Scenario parameter grid
TARGETS_DEG  = [-90, -70, -55, -35, -10, 5, 18, 25, 35, 50, 65, 90]   # degrees
VELOCITIES   = [5, 13, 25, 40, 60, 80]   # deg/s
ACCELERATIONS = [5, 13, 25, 40, 60, 80]  # deg/s²

# Velocity / acceleration used for the reset movement (not recorded)
RESET_VELOCITY = 60     # deg/s
RESET_ACCELERATION = 80 # deg/s²

# Safety timeout per movement (seconds)
SCENARIO_TIMEOUT = 120

# Output path
OUTPUT_DIR = os.path.join(ROOT, "anomaly_detection", "training", "training_data")
OUTPUT_FILE = os.path.join(OUTPUT_DIR, "scenarios.csv")


# ---------------------------------------------------------------------------
# Scenario generation
# ---------------------------------------------------------------------------
def generate_scenarios():
    """Build the full ordered list of scenarios.

    For each batch (start position) every combination of target × velocity ×
    acceleration is produced.  Targets are converted from degrees to radians
    and applied identically to all 6 joints.

    Returns a list of dicts with keys:
        scenario_id, batch_start_pos, target, max_velocity, acceleration
    """
    scenarios = []
    sid = 0
    for start_pos in BATCH_START_POSITIONS:
        for target_deg, vel, acc in itertools.product(TARGETS_DEG, VELOCITIES, ACCELERATIONS):
            target_rad = np.deg2rad(target_deg)
            scenarios.append({
                "scenario_id": sid,
                "batch_start_pos": start_pos,
                "target": [target_rad] * NUM_JOINTS,
                "max_velocity": vel,
                "acceleration": acc,
            })
            sid += 1
    return scenarios


# ---------------------------------------------------------------------------
# Per-scenario state collector
# ---------------------------------------------------------------------------
class ScenarioCollector:
    """Accumulates state messages for one movement and detects completion.

    When record=False the collector only tracks robot mode to detect completion
    (used for unrecorded reset movements).
    """

    def __init__(self, scenario_id, target, max_velocity, acceleration, record=True):
        self.scenario_id = scenario_id
        self.target = target
        self.max_velocity = max_velocity
        self.acceleration = acceleration
        self.record = record
        self.rows: list[list] = []
        self._mode_history: list[str] = []
        self.finished = False

    def on_state(self, _ch, _method, _properties, body):
        mode = body[protocol.RobotArmStateKeys.ROBOT_MODE]
        q_actual = body[protocol.RobotArmStateKeys.Q_ACTUAL]
        q_target = body[protocol.RobotArmStateKeys.Q_TARGET]
        timestamp = body[protocol.RobotArmStateKeys.TIMESTAMP]

        self._mode_history.append(mode)

        if self.record:
            # Flat row: scenario_id, timestamp, q_actual_0‥5, q_target_0‥5, vel, acc
            row = [self.scenario_id, timestamp]
            row.extend(q_actual)
            row.extend(q_target)
            row.extend([self.max_velocity, self.acceleration])
            self.rows.append(row)

        # Detect completion: was running → now idle
        if len(self._mode_history) > 2:
            was_running = any(m.lower() != "idle" for m in self._mode_history)
            if was_running and mode.lower() == "idle":
                self.finished = True


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def run_movement(rmq, current, target, velocity, acceleration, scenario_id=None, record=True, timeout=SCENARIO_TIMEOUT):
    """Send LOAD_PROGRAM + PLAY, wait for completion, return the collector."""
    collector = ScenarioCollector(
        scenario_id=scenario_id,
        target=target,
        max_velocity=velocity,
        acceleration=acceleration,
        record=record,
    )
    current["collector"] = collector

    rmq.send_message(
        routing_key=protocol.ROUTING_KEY_CTRL,
        message={
            protocol.CtrlMsgKeys.TYPE: protocol.CtrlMsgFields.LOAD_PROGRAM,
            protocol.CtrlMsgKeys.JOINT_POSITIONS: [target],
            protocol.CtrlMsgKeys.MAX_VELOCITY: velocity,
            protocol.CtrlMsgKeys.ACCELERATION: acceleration,
        },
    )
    rmq.send_message(
        routing_key=protocol.ROUTING_KEY_CTRL,
        message={protocol.CtrlMsgKeys.TYPE: protocol.CtrlMsgFields.PLAY},
    )

    t0 = time.time()
    while not collector.finished:
        rmq.connection.process_data_events(time_limit=1)
        if time.time() - t0 > timeout:
            label = f"scenario {scenario_id}" if record else "reset movement"
            print(f"  WARNING: {label} timed out after {timeout}s")
            break
    return collector


def load_completed_ids(filepath) -> set[int]:
    """Return the set of scenario IDs already present in the CSV.

    Rows are written atomically per scenario (writerows + flush), so any ID
    found in the file belongs to a fully completed scenario.
    """
    completed = set()
    if not Path(filepath).exists():
        return completed
    with open(filepath, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                completed.add(int(row["scenario_id"]))
            except (KeyError, ValueError):
                pass
    return completed


def main():
    scenarios = generate_scenarios()
    total = len(scenarios)
    print(f"Generated {total} scenarios across {len(BATCH_START_POSITIONS)} batches")
    print(f"  ({len(TARGETS_DEG)} targets × {len(VELOCITIES)} velocities × "
          f"{len(ACCELERATIONS)} accelerations × {len(BATCH_START_POSITIONS)} batches)")

    # Connect to RabbitMQ
    rmq = Rabbitmq(
        ip="localhost",
        port=5672,
        username="incubator",
        password="incubator",  # noqa: S106
        vhost="/",
        exchange="UR3E_AMQP",
        type="topic",
    )
    rmq.connect_to_server()
    print("Connected to RabbitMQ")

    # Prepare CSV – detect already-completed scenarios for resume support
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    completed_ids = load_completed_ids(OUTPUT_FILE)
    remaining = [s for s in scenarios if s["scenario_id"] not in completed_ids]
    if completed_ids:
        print(f"Resuming: {len(completed_ids)} scenarios already done, "
              f"{len(remaining)} remaining")
    else:
        print(f"Starting fresh – {total} scenarios to run")

    if not remaining:
        print("All scenarios already completed. Nothing to do.")
        rmq.close()
        return

    header = (
        ["scenario_id", "timestamp"]
        + [f"q_actual_{i}" for i in range(NUM_JOINTS)]
        + [f"q_target_{i}" for i in range(NUM_JOINTS)]
        + ["max_velocity", "acceleration"]
    )
    # Append if file exists (resuming), otherwise create with header
    file_mode = "a" if completed_ids else "w"

    # Single shared subscriber; the active collector is swapped per movement
    current = {"collector": None}

    def _on_state(ch, method, properties, body):
        c = current["collector"]
        if c is not None:
            c.on_state(ch, method, properties, body)

    rmq.subscribe(
        routing_key=protocol.ROUTING_KEY_STATE,
        on_message_callback=_on_state,
    )

    with open(OUTPUT_FILE, file_mode, newline="") as f:
        writer = csv.writer(f)
        if file_mode == "w":
            writer.writerow(header)

        for scenario in remaining:
            sid = scenario["scenario_id"]
            start_pos = scenario["batch_start_pos"]

            # ── 1. Reset robot to batch start position (not recorded) ──────
            reset_target = [start_pos] * NUM_JOINTS
            run_movement(
                rmq, current,
                target=reset_target,
                velocity=RESET_VELOCITY,
                acceleration=RESET_ACCELERATION,
                record=False,
            )

            # ── 2. Run the actual scenario (recorded) ───────────────────────
            collector = run_movement(
                rmq, current,
                target=scenario["target"],
                velocity=scenario["max_velocity"],
                acceleration=scenario["acceleration"],
                scenario_id=sid,
                record=True,
            )

            writer.writerows(collector.rows)
            f.flush()

            done_count = len(completed_ids) + remaining.index(scenario) + 1
            if done_count % 50 == 0 or done_count == 1:
                print(f"  [{done_count}/{total}] scenario {sid} – collected {len(collector.rows)} steps")

    rmq.close()
    print(f"\nDone – {total} scenarios saved to {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
