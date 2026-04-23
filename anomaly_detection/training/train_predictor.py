"""Train per-joint error predictors for scenario-level anomaly detection.

The DT motion model already predicts what a healthy robot trajectory should
look like.  The DT-PT error on normal data follows a predictable pattern
that depends on the scenario configuration (joint distances, velocity,
acceleration).

This script trains 6 independent Gradient Boosted Tree (GBT) regressors --
one per joint -- that learn:

    condition  →  expected mean |error|

At detection time, any joint where ``actual_error - predicted_error`` exceeds
a calibrated threshold is flagged as anomalous.  Because the predictor never
sees the actual error at inference, a faulty robot that produces larger-than-
expected errors will be caught.

Features  (14-D condition vector)
---------------------------------
    |target_j − start_j|   (6)   travel distances
    sign(target_j − start_j) (6) directions
    max_velocity             (1)
    acceleration             (1)

Targets  (6 values, one per joint)
----------------------------------
    mean |DT − PT error| across all steps in the scenario

Saved artefacts  (anomaly_detection/trained_model/)
---------------------------------------------------
    predictor.joblib   - list of 6 fitted GBT regressors
    thresholds.npy     - per-joint anomaly thresholds (6,)

Usage
-----
    python anomaly_detection/training/train_predictor.py
"""

import os
import sys

import joblib
import numpy as np
import pandas as pd
from sklearn.ensemble import GradientBoostingRegressor
from sklearn.model_selection import train_test_split

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, os.pardir))
sys.path.insert(0, ROOT)

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
DATA_FILE = os.path.join(ROOT, "anomaly_detection", "training", "data", "model_errors.csv")
SAVE_DIR = os.path.join(ROOT, "anomaly_detection", "trained_model")

NUM_JOINTS = 6
COND_DIM = 14  # 6 distances + 6 directions + vel + acc

# Hyper-parameters
THRESHOLD_PERCENTILE = 99.9
VAL_SPLIT = 0.2
SEED = 42

# GBT hyper-parameters (per-joint regressor)
GBT_PARAMS = dict(
    n_estimators=500,
    max_depth=5,
    learning_rate=0.05,
    subsample=0.8,
    min_samples_leaf=5,
    random_state=SEED,
)


# ---------------------------------------------------------------------------
# Feature engineering  (scenario-level aggregation)
# ---------------------------------------------------------------------------
def build_features(df: pd.DataFrame):
    """Return (condition, error) arrays -- one row per scenario.

    Parameters
    ----------
    df : DataFrame with columns scenario_id, start_0..5, target_0..5,
         max_velocity, acceleration, error_joint_0..5

    Returns
    -------
    cond   : (S, 14)  condition vectors
    errors : (S, 6)   mean |DT-PT error| per joint
    sids   : (S,)     scenario ids
    """
    start_cols = [f"start_{j}" for j in range(NUM_JOINTS)]
    target_cols = [f"target_{j}" for j in range(NUM_JOINTS)]
    error_cols = [f"error_joint_{j}" for j in range(NUM_JOINTS)]

    grouped = df.groupby("scenario_id")

    # Mean absolute error per joint per scenario
    mean_abs_err = grouped[error_cols].apply(lambda x: x.abs().mean()).reset_index()

    # Scenario config (constant within a scenario)
    config = grouped.first().reset_index()

    sids = config["scenario_id"].values
    starts = config[start_cols].values
    targets = config[target_cols].values

    travel_dist = np.abs(targets - starts)
    direction = np.sign(targets - starts)
    vel = config["max_velocity"].values.reshape(-1, 1)
    acc = config["acceleration"].values.reshape(-1, 1)

    cond = np.hstack([travel_dist, direction, vel, acc]).astype(np.float32)  # (S, 14)
    errors = mean_abs_err[error_cols].values.astype(np.float32)              # (S, 6)

    return cond, errors, sids


# ---------------------------------------------------------------------------
# Training
# ---------------------------------------------------------------------------
def train_regressors(X_train, Y_train, X_val, Y_val):
    """Train 6 independent GBT regressors and report per-joint R² scores.

    Returns
    -------
    models : list of 6 fitted GradientBoostingRegressor
    """
    models = []
    for j in range(NUM_JOINTS):
        print(f"  Joint {j} ... ", end="", flush=True)
        reg = GradientBoostingRegressor(**GBT_PARAMS)
        reg.fit(X_train, Y_train[:, j])

        r2_train = reg.score(X_train, Y_train[:, j])
        r2_val = reg.score(X_val, Y_val[:, j])
        print(f"R²  train={r2_train:.4f}  val={r2_val:.4f}")
        models.append(reg)
    return models


def predict_all(models, X):
    """Run all 6 regressors → (N, 6) predictions."""
    return np.column_stack([m.predict(X) for m in models])


# ---------------------------------------------------------------------------
# Threshold computation
# ---------------------------------------------------------------------------
def compute_thresholds(models, X, Y_actual):
    """Thresholds on (actual - predicted) residuals.

    Uses the THRESHOLD_PERCENTILE of the *positive* residual per joint
    over the full dataset.  Positive residual = actual error higher than
    predicted (the direction we care about).
    """
    Y_pred = predict_all(models, X)
    residuals = Y_actual - Y_pred  # positive = worse than expected

    thresholds = np.array([
        np.percentile(residuals[:, j], THRESHOLD_PERCENTILE)
        for j in range(NUM_JOINTS)
    ])
    return thresholds, residuals


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    np.random.seed(SEED)

    print(f"Loading data from:\n  {DATA_FILE}\n")
    df = pd.read_csv(DATA_FILE)
    n_scenarios = df["scenario_id"].nunique()
    print(f"  {len(df)} rows, {n_scenarios} scenarios\n")

    # Aggregate to scenario level
    cond_all, err_all, sids_all = build_features(df)
    print(f"  After aggregation: {len(sids_all)} scenario-level samples\n")

    # Train/val split
    X_train, X_val, Y_train, Y_val = train_test_split(
        cond_all, err_all, test_size=VAL_SPLIT, random_state=SEED,
    )
    print(f"  Train: {len(X_train)}  Val: {len(X_val)}\n")

    # Train per-joint regressors
    print("Training GBT regressors ...")
    models = train_regressors(X_train, Y_train, X_val, Y_val)

    # Compute thresholds on FULL dataset (train + val)
    thresholds, residuals = compute_thresholds(models, cond_all, err_all)

    print(f"\nPer-joint thresholds ({THRESHOLD_PERCENTILE}th pctl of residual):")
    for j in range(NUM_JOINTS):
        print(f"  joint {j}: {thresholds[j]:.6f} rad")

    # Summary stats
    Y_pred_all = predict_all(models, cond_all)
    for j in range(NUM_JOINTS):
        mae = np.mean(np.abs(err_all[:, j] - Y_pred_all[:, j]))
        print(f"  joint {j}: MAE = {mae:.6f} rad  |  "
              f"mean actual = {err_all[:, j].mean():.6f}  "
              f"mean predicted = {Y_pred_all[:, j].mean():.6f}")

    # Save
    os.makedirs(SAVE_DIR, exist_ok=True)

    model_path = os.path.join(SAVE_DIR, "predictor.joblib")
    joblib.dump(models, model_path)

    thresh_path = os.path.join(SAVE_DIR, "thresholds.npy")
    np.save(thresh_path, thresholds)

    print(f"\nSaved to {SAVE_DIR}/")
    print(f"  predictor.joblib  ({os.path.getsize(model_path) / 1024:.1f} KB)")
    print(f"  thresholds.npy")


if __name__ == "__main__":
    main()
