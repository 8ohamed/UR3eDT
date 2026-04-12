# 2D Visualization

## What this module does

The `_2D_visual` module provides a live dashboard that compares the PT joint positions against the calibrated digital model in real time.

It has two background services:

- **PTRecorderService** listens to PT state messages over RabbitMQ and writes the actual joint positions to InfluxDB while the robot is moving.
- **ModelTrackerService** on each motion, runs the calibrated model and writes predicted joint positions alongside the observed data.

Grafana reads both data streams from InfluxDB and displays them on the **UR3e Model vs Observed** dashboard one panel per joint, with PT data as blue dots and model predictions as a red line.

## Setup and starting the visualization

**1. Start InfluxDB and Grafana**
From the `_2D_visual/` folder:

```bash
docker compose up -d
```

Grafana will be available at [http://localhost:3000](http://localhost:3000) (user: `ur3e`, password: `ur3e_password`).
InfluxDB will be available at [http://localhost:8086](http://localhost:8086).

**2. Start the DT visualization services**
From the workspace root, make sure RabbitMQ and the PT mockup are already running, then:

```bash
python -m _2D_visual.main
```

This starts both services and keeps them running in the background.

**3. Open the dashboard**
In Grafana, navigate to **Dashboards → UR3e Model vs Observed**. The panels update every 2 seconds as the robot moves.

## Challenges

### 1. The model is ready instantly, but the PT data arrives piece by piece

The model is a mathematical prediction, we can compute the entire trajectory (all time steps, all joints) in one go before the robot even starts moving. The PT data, on the other hand, arrives as a stream of messages at 20 Hz while the robot is moving. This creates an asymmetry: the model already knows where joint 0 will be at t=1.5s, but the PT hasn't reported that data point yet.

**What we did:** We pre-compute the full model trajectory when the motion command (`LOAD_PROGRAM`) is received, before the first `Running` state arrives. We store the result as interpolators (one per joint). Then, as each PT state message comes in, we sample the model at that exact elapsed time and write both the PT value and the model value to InfluxDB simultaneously. This keeps both curves advancing in lockstep on the dashboard.

**What could be better:** Right now the model is written point-by-point in sync with the PT, which means the full model curve only appears as the motion completes. An alternative would be to write the entire model trajectory to InfluxDB upfront as soon as `LOAD_PROGRAM` is received, so the predicted path is visible before motion starts useful for a "plan vs. actual" view.

---

### 2. We do not know how many PT steps there will be

The model runs at a fine 2 ms simulation step and produces hundreds of time points. The PT publishes at 20 Hz (one message every 50 ms). We do not know in advance how long the motion will last, it depends on the distance, velocity, and acceleration commanded. So there is no fixed number of PT data points to expect.

**What we did:** We use `scipy.interp1d` to turn the dense model output into a continuous function. When a PT message arrives at elapsed time `t`, we simply call `q_interp[j](t)` and get the model's prediction at that exact moment, regardless of how many total steps the PT will send. `fill_value="extrapolate"` handles cases where the PT runs slightly longer than the model predicted.

**What could be better:** Extrapolation beyond the model's end time is a silent assumption, the joint is "at target" and the extrapolated value is flat, which is correct in practice but not guaranteed. A cleaner approach would be to clamp `elapsed` to the model's `t_end` rather than extrapolating.

---

### 3. Two separate services on two separate clocks

The PT recorder and the model tracker run in separate threads. Both need to write data with matching wall-clock timestamps so the curves line up on the Grafana time axis. Early on they each captured `time.time_ns()` independently when motion started, but one thread fired ~150 ms before the other, causing the two curves to appear shifted horizontally even though they represented the same physical motion.

**What we did:** Both services now compute a single stable offset once when the first state message arrives during idle: `offset = wall_ns - robot_ns`. From then on, every timestamp is computed as `offset + int(robot_timestamp * 1e9)`. Because the robot clock and the wall clock both measure real seconds (just different epochs), the offset is stable and both services compute essentially the same value, regardless of which specific message they happen to calibrate on.

**What could be better:** The cleanest solution would be to share a single clock offset computed once in `main.py` and passed into both services at construction time, removing the dependency on both services independently agreeing on the offset.
