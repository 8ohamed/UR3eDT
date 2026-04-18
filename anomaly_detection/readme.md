# Anomaly Detection

Detects anomalous robot behaviour by comparing the PT's observed joint trajectories
against what the DT motion model predicts for the same scenario conditions.  A
per-joint Gradient Boosted Tree (GBT) regressor learns the expected mean absolute
DT–PT error as a function of scenario configuration (travel distances, directions,
velocity, acceleration).  A joint is flagged as anomalous when the actual error
exceeds the prediction by more than a calibrated threshold.

---

## Folder structure

```
anomaly_detection/
├── error.py                          # Core: runs DT simulation for one scenario
│                                     #   and returns per-step, per-joint errors
├── trained_model/
│   ├── predictor.joblib              # 6 fitted GBT regressors (one per joint)
│   └── thresholds.npy               # Per-joint residual thresholds (rad)
└── training/
    ├── collect_training_data/
    │   └── collect_PT_scenarios.py   # Step 1: record PT scenarios via RabbitMQ
    ├── data/
    │   ├── scenarios.csv             # Raw PT recordings (output of step 1)
    │   └── model_errors.csv          # Per-step DT–PT errors  (output of step 2)
    ├── compute_model_error.py        # Step 2: compute DT–PT errors for all scenarios
    ├── train_predictor.py            # Step 3: train GBT regressors & compute thresholds
    └── detect_anomaly.py             # Step 4: run live or file-based detection
```

---

## Training a model from scratch

Run the four steps below in order from the **project root**.

> **Prerequisites for steps 1 and 4:** the UR3e mockup and RabbitMQ must be
> running before executing those scripts.

### Step 1: Collect PT training data

```bash
python anomaly_detection/training/collect_training_data/collect_PT_scenarios.py
```

Executes 3 024 structured scenarios on the mockup (7 start positions × 12 targets
× 6 velocities × 6 accelerations) and logs every state message to:

```bash
anomaly_detection/training/data/scenarios.csv
```

### Step 2: Compute DT–PT errors

```bash
python anomaly_detection/training/compute_model_error.py
```

Reads `scenarios.csv`, runs the DT simulation for every scenario, and writes the
per-step, per-joint errors to:

```bash
anomaly_detection/training/data/model_errors.csv
```

### Step 3: Train the GBT predictors

```bash
python anomaly_detection/training/train_predictor.py
```

Trains 6 independent GBT regressors (condition → expected mean |error| per joint)
and computes per-joint residual thresholds at the 99.9th percentile of the training
residuals.  Saves the artefacts to:

```bash
anomaly_detection/trained_model/predictor.joblib
anomaly_detection/trained_model/thresholds.npy
```

---

## Running the anomaly detector

### Live mode (RabbitMQ)

```bash
python anomaly_detection/training/detect_anomaly.py
```

Subscribes to the `robotarm.pt.state` topic on RabbitMQ.  Each time the robot
completes a movement (transitions from `Running` to `Idle`), the buffered state
messages are classified in a background thread.  Up to five scenarios are
classified concurrently.

### File mode (pre-recorded CSV)

```bash
python anomaly_detection/training/detect_anomaly.py <path/to/scenario.csv>
```

Reads a pre-recorded scenario CSV and classifies every scenario in it, printing a
per-scenario verdict and a summary line at the end.

---

## Updating after the motion model is re-calibrated

If the DT motion model parameters (`parameters.json`) are updated (e.g. after a
new calibration run), the GBT predictors must be retrained because the DT–PT
error distribution changes with the model.  Repeat **steps 2 and 3** only — there
is no need to re-collect PT data:

```bash
python anomaly_detection/training/compute_model_error.py
python anomaly_detection/training/train_predictor.py
```

This recomputes the errors using the new model parameters and retrains the
predictors and thresholds accordingly.
