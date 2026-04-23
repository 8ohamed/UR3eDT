"""Deterministic per-step error between a PT scenario and the DT motion model.

Given a group of rows from the scenarios CSV (one scenario_id), this module
extracts the start position, target, and motion constraints, runs the DT
simulation with a time-step and duration chosen so that the number of
simulated steps exactly matches the number of recorded PT steps, and returns
the per-step, per-joint error (PT observed vs DT predicted).

Output convention
-----------------
compute_scenario_error() returns ``(header_row, errors)`` where:

* ``header_row`` 1-D array of length 14:
      [start_0 … start_5, target_0 … target_5, max_velocity, acceleration]
  This row fully identifies the scenario configuration.

* ``errors`` 2-D array of shape (N_steps, 6):
      errors[i, j] = q_actual_j[i] vs q_predicted_j[i]   (radians)
"""

import os
import sys
import json

import numpy as np
import pandas as pd

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
sys.path.insert(0, ROOT)

from models.model import MotionModel

NUM_JOINTS = 6
PARAMS_PATH = os.path.join(ROOT, "models", "parameters.json")
_FALLBACK_DT = 1.0 / 20.0   # 20 Hz – matches the observed ~0.05 s step


def load_params(params_path: str = PARAMS_PATH) -> dict:
    """Load model parameters from *parameters.json*."""
    with open(params_path, "r") as f:
        return json.load(f)


def _infer_dt(timestamps: np.ndarray) -> float:
    """Estimate the sampling interval (seconds) from an array of timestamps."""
    if len(timestamps) < 2:
        return _FALLBACK_DT
    diffs = np.diff(timestamps.astype(float))
    dt = float(np.median(diffs))
    return dt if dt > 0 else _FALLBACK_DT


# Public API
def compute_scenario_error(
    scenario_df: pd.DataFrame,
    params: dict | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Compute the per-step error between a PT scenario and the DT model.

    The simulation is configured so that it produces **exactly the same number
    of time steps** as the physical twin recording.  This is achieved by
    deriving ``dt`` from the PT timestamps and setting
    ``t_end = (N - 1) * dt``, which makes ``simulate_joint_motion`` return
    ``int(ceil(t_end / dt)) + 1 = N`` steps.

    Parameters
    ----------
    scenario_df:
        All rows for a single ``scenario_id`` from the scenarios CSV.
        Required columns: ``timestamp``, ``q_actual_{0..5}``,
        ``q_target_{0..5}``, ``max_velocity``, ``acceleration``.
    params:
        DT model parameters (``vel_scale``, ``acc_scale``, ``smooth_tau``).
        Loaded from *models/parameters.json* when *None*.

    Returns
    -------
    header_row : np.ndarray, shape (14,)
        ``[start_0…5, target_0…5, max_velocity, acceleration]``
    errors : np.ndarray, shape (N_steps, 6)
        ``q_actual vs q_predicted`` per step per joint (radians).
    """
    if params is None:
        params = load_params()

    df = scenario_df.reset_index(drop=True)

    # ── Extract scenario configuration from the first row ──────────────────
    q_actual_cols = [f"q_actual_{i}" for i in range(NUM_JOINTS)]
    q_target_cols = [f"q_target_{i}" for i in range(NUM_JOINTS)]

    q0 = df[q_actual_cols].iloc[0].to_numpy(dtype=float)
    # Use the last row's q_target: during movement q_target is the trajectory
    # waypoint, so the final row gives the commanded destination.
    q_target = df[q_target_cols].iloc[-1].to_numpy(dtype=float)
    max_velocity = float(df["max_velocity"].iloc[0])
    acceleration = float(df["acceleration"].iloc[0])

    # Header row: full scenario configuration
    header_row = np.concatenate([q0, q_target, [max_velocity, acceleration]])

    # ── PT observed positions ───────────────────────────────────────────────
    q_actual = df[q_actual_cols].to_numpy(dtype=float)   # shape (N, 6)
    N = len(q_actual)

    if N == 0:
        return header_row, np.empty((0, NUM_JOINTS))

    # ── Match simulation length to PT step count ────────────────────────────
    timestamps = df["timestamp"].to_numpy(dtype=float)
    dt = _infer_dt(timestamps)

    # simulate_joint_motion uses:  steps = int(ceil(t_end / dt)) + 1
    # Setting t_end = (N-1)*dt guarantees steps == N.
    t_end = (N - 1) * dt

    # ── Run DT model ────────────────────────────────────────────────────────
    model = MotionModel()
    _, q_log, _, _ = model.simulate_joint_motion(
        q0=q0,
        q_target=q_target,
        max_velocity_deg=max_velocity,
        acceleration_deg=acceleration,
        dt=dt,
        t_end=t_end,
        vel_scale=params["vel_scale"],
        acc_scale=params["acc_scale"],
        smooth_tau=params.get("smooth_tau", 0.0),
    )

    # ── Compute errors (align lengths as a safety measure) ──────────────────
    min_len = min(N, len(q_log))
    errors = q_actual[:min_len] - q_log[:min_len]   # (N_steps, 6)

    return header_row, errors
