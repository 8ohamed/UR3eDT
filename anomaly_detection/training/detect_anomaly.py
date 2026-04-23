"""Detect anomalies in PT scenarios using trained GBT error predictors.

The predictor maps scenario conditions → expected mean |DT-PT error| per
joint.  A joint is flagged as anomalous when:

    actual_mean_error  -  predicted_mean_error  >  threshold

Two modes
---------
Live (default, no arguments):
    Subscribes to RabbitMQ and classifies each completed movement in a
    background thread (up to MAX_CLASSIFIERS concurrent).

File (positional argument = path to CSV):
    Reads a pre-recorded scenario CSV and classifies every scenario.

Usage
-----
    python anomaly_detection/training/detect_anomaly.py
    python anomaly_detection/training/detect_anomaly.py <scenario_csv>
"""

import os
import sys
import threading
from concurrent.futures import ThreadPoolExecutor

import joblib
import numpy as np
import pandas as pd

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, os.pardir))
sys.path.insert(0, ROOT)

from communication import protocol
from communication.rabbitmq import Rabbitmq
from anomaly_detection.error import compute_scenario_error, load_params, NUM_JOINTS
from _2D_visual.influxdb.client import InfluxClient

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
MAX_CLASSIFIERS = 5

RABBITMQ_HOST = "localhost"
RABBITMQ_PORT = 5672
RABBITMQ_USER = "ur3e"
RABBITMQ_PASS = "ur3e"   # noqa: S105
RABBITMQ_VHOST = "/"
RABBITMQ_EXCHANGE = "UR3E_AMQP"

INFLUXDB_URL = "http://localhost:8086"
INFLUXDB_TOKEN = "ur3e-dt-token"  # noqa: S105
INFLUXDB_ORG = "ur3e_dt"
INFLUXDB_BUCKET = "ur3e"

MEASUREMENT_STATUS = "joint_anomaly_status"
CONFIDENCE_LABELS = ["Normal", "Anomaly (Low)", "Anomaly (Medium)", "Anomaly (High)"]

# ---------------------------------------------------------------------------
# Paths to trained artefacts
# ---------------------------------------------------------------------------
MODEL_DIR = os.path.join(ROOT, "anomaly_detection", "trained_model")
PREDICTOR_PATH = os.path.join(MODEL_DIR, "predictor.joblib")
THRESH_PATH = os.path.join(MODEL_DIR, "thresholds.npy")


# ---------------------------------------------------------------------------
# Load trained model & artefacts
# ---------------------------------------------------------------------------
def load_model():
    """Return (models_list, thresholds).

    models_list : list of 6 fitted GBT regressors
    thresholds  : (6,) per-joint residual thresholds
    """
    models = joblib.load(PREDICTOR_PATH)
    thresholds = np.load(THRESH_PATH)  # (6,)
    return models, thresholds


# ---------------------------------------------------------------------------
# Build scenario-level condition vector
# ---------------------------------------------------------------------------
def _build_condition(header_row: np.ndarray) -> np.ndarray:
    """Return (1, 14) condition vector for one scenario.

    header_row layout: [start_0..5, target_0..5, max_velocity, acceleration]
    """
    starts = header_row[:6]
    targets = header_row[6:12]
    vel = header_row[12]
    acc = header_row[13]

    travel_dist = np.abs(targets - starts)
    direction = np.sign(targets - starts)

    cond = np.concatenate([travel_dist, direction, [vel, acc]])
    return cond.astype(np.float32).reshape(1, -1)  # (1, 14)


def _predict_all(models, X):
    """Run all 6 regressors → (N, 6) predictions."""
    return np.column_stack([m.predict(X) for m in models])


# ---------------------------------------------------------------------------
# Core classification  (scenario-level)
# ---------------------------------------------------------------------------
def detect_scenario(
    scenario_df: pd.DataFrame,
    models: list,
    thresholds: np.ndarray,
    params: dict | None = None,
) -> dict:
    """Classify one scenario.  Returns a dict with per-joint verdicts.

    Steps:
    1. Compute per-step DT-PT errors via the deterministic model.
    2. Aggregate to mean |error| per joint.
    3. Predict expected mean |error| from conditions using GBT models.
    4. Flag joints where (actual - predicted) > threshold.

    Returns
    -------
    dict with keys:
        header           -- scenario config (14,)
        mean_abs_error   -- actual mean |DT-PT error| per joint (6,)
        predicted_error  -- predicted mean |error| per joint (6,)
        residual         -- actual - predicted per joint (6,)
        anomalous_joints -- list of ints flagged as anomalous
        is_anomaly       -- bool
        n_steps          -- number of time steps
    """
    header_row, errors = compute_scenario_error(scenario_df, params)
    n_steps = len(errors)

    empty = {
        "header": header_row,
        "mean_abs_error": np.zeros(6),
        "predicted_error": np.zeros(6),
        "residual": np.zeros(6),
        "anomalous_joints": [],
        "is_anomaly": False,
        "n_steps": 0,
    }
    if n_steps == 0:
        return empty

    # Scenario-level aggregation: mean |error| per joint
    mean_abs_err = np.abs(errors).mean(axis=0).astype(np.float32)  # (6,)

    # Predict expected error from conditions only
    cond = _build_condition(header_row)  # (1, 14)
    predicted = _predict_all(models, cond).flatten()  # (6,)

    # Residual: positive = worse than expected
    residual = mean_abs_err - predicted

    anomalous_joints = [
        j for j in range(NUM_JOINTS)
        if residual[j] > thresholds[j]
    ]

    return {
        "header": header_row,
        "mean_abs_error": mean_abs_err,
        "predicted_error": predicted,
        "residual": residual,
        "anomalous_joints": anomalous_joints,
        "is_anomaly": len(anomalous_joints) > 0,
        "n_steps": n_steps,
    }


def _print_result(sid: int, result: dict) -> None:
    n = result["n_steps"]
    if not result["is_anomaly"]:
        print(f"  [Scenario {sid}] {n} steps -> NORMAL")
    else:
        joints = result["anomalous_joints"]
        residuals = result["residual"]
        details = ", ".join(
            f"j{j} residual={residuals[j]:.5f}" for j in joints
        )
        print(f"  [Scenario {sid}] {n} steps -> ANOMALY  (joints: {joints})  [{details}]")


# ---------------------------------------------------------------------------
# Live RabbitMQ listener
# ---------------------------------------------------------------------------
class _ScenarioBuffer:
    def __init__(self, scenario_id: int):
        self.scenario_id = scenario_id
        self.rows: list[dict] = []

    def append(self, body: dict) -> None:
        timestamp = body[protocol.RobotArmStateKeys.TIMESTAMP]
        q_actual = body[protocol.RobotArmStateKeys.Q_ACTUAL]
        q_target = body[protocol.RobotArmStateKeys.Q_TARGET]

        raw_vel = body.get(protocol.RobotArmStateKeys.JOINT_MAX_SPEED, 0)
        raw_acc = body.get(protocol.RobotArmStateKeys.JOINT_MAX_ACCELERATION, 0)
        max_velocity = float(np.mean(raw_vel) if isinstance(raw_vel, (list, np.ndarray)) else raw_vel)
        acceleration = float(np.mean(raw_acc) if isinstance(raw_acc, (list, np.ndarray)) else raw_acc)

        row = {
            "scenario_id": self.scenario_id,
            "timestamp": timestamp,
            "max_velocity": max_velocity,
            "acceleration": acceleration,
        }
        for i, v in enumerate(q_actual):
            row[f"q_actual_{i}"] = v
        for i, v in enumerate(q_target):
            row[f"q_target_{i}"] = v
        self.rows.append(row)

    def to_dataframe(self) -> pd.DataFrame:
        return pd.DataFrame(self.rows)


class LiveAnomalyDetector:
    def __init__(self, models, thresholds, params, influx=None):
        self._models = models
        self._thresholds = thresholds
        self._params = params
        self._influx = influx

        self._executor = ThreadPoolExecutor(max_workers=MAX_CLASSIFIERS)
        self._lock = threading.Lock()
        self._buffer: _ScenarioBuffer | None = None
        self._last_mode: str | None = None
        self._scenario_counter = 0
        self._confidence = np.zeros(NUM_JOINTS, dtype=int)

        if self._influx:
            self._write_status()

    def on_state(self, _ch, _method, _properties, body: dict) -> None:
        mode: str = body[protocol.RobotArmStateKeys.ROBOT_MODE]

        with self._lock:
            prev = self._last_mode

            if (prev is not None
                    and prev.lower() != "idle"
                    and mode.lower() == "idle"):
                if self._buffer is not None and self._buffer.rows:
                    buf = self._buffer
                    self._buffer = None
                    self._executor.submit(self._classify, buf)

            if (mode.lower() != "idle"
                    and (prev is None or prev.lower() == "idle")):
                self._scenario_counter += 1
                self._buffer = _ScenarioBuffer(self._scenario_counter)
                print(f"\n[Scenario {self._scenario_counter}] started - collecting steps ...")

            if mode.lower() != "idle" and self._buffer is not None:
                self._buffer.append(body)

            self._last_mode = mode

    def _classify(self, buf: _ScenarioBuffer) -> None:
        sid = buf.scenario_id
        df = buf.to_dataframe()
        print(f"  [Scenario {sid}] movement finished ({len(df)} steps) - classifying ...")
        try:
            result = detect_scenario(
                df, self._models, self._thresholds, self._params,
            )
            _print_result(sid, result)
            self._update_confidence(result)
        except Exception as exc:
            print(f"  [Scenario {sid}] classification error: {exc}")

    def _update_confidence(self, result: dict) -> None:
        """Step confidence up on anomaly, down on normal, per joint."""
        with self._lock:
            for j in range(NUM_JOINTS):
                if j in result["anomalous_joints"]:
                    self._confidence[j] = min(self._confidence[j] + 1, 3)
                else:
                    self._confidence[j] = max(self._confidence[j] - 1, 0)
        self._write_status()
        self._print_confidence()

    def _write_status(self) -> None:
        """Write current per-joint confidence level to InfluxDB."""
        if not self._influx:
            return
        fields = {}
        for j in range(NUM_JOINTS):
            fields[f"status_j{j}"] = int(self._confidence[j])
        self._influx.write_point(
            measurement=MEASUREMENT_STATUS,
            fields=fields,
        )

    def _print_confidence(self) -> None:
        parts = []
        for j in range(NUM_JOINTS):
            label = CONFIDENCE_LABELS[self._confidence[j]]
            parts.append(f"j{j}={label}")
        print(f"  Confidence: {', '.join(parts)}")

    def shutdown(self) -> None:
        self._executor.shutdown(wait=True)
        if self._influx:
            self._influx.close()


# ---------------------------------------------------------------------------
# Entry points
# ---------------------------------------------------------------------------
def _run_live() -> None:
    models, thresholds = load_model()
    params = load_params()

    influx = InfluxClient(
        url=INFLUXDB_URL, token=INFLUXDB_TOKEN,
        org=INFLUXDB_ORG, bucket=INFLUXDB_BUCKET,
    )
    influx.connect()

    detector = LiveAnomalyDetector(models, thresholds, params, influx=influx)

    rmq = Rabbitmq(
        ip=RABBITMQ_HOST,
        port=RABBITMQ_PORT,
        username=RABBITMQ_USER,
        password=RABBITMQ_PASS,
        vhost=RABBITMQ_VHOST,
        exchange=RABBITMQ_EXCHANGE,
        type="topic",
    )
    rmq.connect_to_server()
    rmq.subscribe(
        routing_key=protocol.ROUTING_KEY_STATE,
        on_message_callback=detector.on_state,
    )
    print("Anomaly detector LIVE  (Ctrl+C to stop)\n")
    try:
        rmq.start_consuming()
    except KeyboardInterrupt:
        print("\nShutting down ...")
    finally:
        detector.shutdown()


def _run_file(csv_path: str) -> None:
    if not os.path.exists(csv_path):
        alt_path = os.path.join(ROOT, csv_path)
        if os.path.exists(alt_path):
            csv_path = alt_path
        else:
            print(f"ERROR: File not found: {csv_path}")
            sys.exit(1)

    print(f"Loading PT data from:\n  {csv_path}\n")
    df = pd.read_csv(csv_path)

    models, thresholds = load_model()
    params = load_params()

    scenario_ids = df["scenario_id"].unique()
    print(f"Found {len(scenario_ids)} scenario(s).\n")

    n_anomalous = 0
    for sid in scenario_ids:
        result = detect_scenario(
            df[df["scenario_id"] == sid],
            models, thresholds, params,
        )
        _print_result(sid, result)
        if result["is_anomaly"]:
            n_anomalous += 1

    print(f"\nSummary: {n_anomalous} / {len(scenario_ids)} scenarios flagged as ANOMALY.")


def main() -> None:
    if len(sys.argv) >= 2:
        _run_file(sys.argv[1])
    else:
        _run_live()


if __name__ == "__main__":
    main()
