"""Service that runs the calibrated motion model and writes predictions to InfluxDB.

Workflow per motion (Idle → Running → Idle):
    1. On LOAD_PROGRAM, pre-compute the full trajectory.
    2. On the first "Running" state, batch-write the entire model trajectory
       to InfluxDB at once (with future timestamps).  Grafana reveals each
       point as wall-clock "now" reaches it, so the red line advances in
       real-time — perfectly synchronised with PT data.
    3. While the motion is active, write tracking error per PT state message.
    4. On transition back to "Idle", reset and wait for the next motion.

Measurements written:
    ``model_prediction`` predicted position / velocity / acceleration per joint
    ``tracking_error``    position error  (model vs observed)  per joint
"""

import logging
import math
import time
import numpy as np
from scipy.interpolate import interp1d
from influxdb_client import Point, WritePrecision

from communication.rabbitmq import Rabbitmq
from communication.protocol import (
    ROUTING_KEY_CTRL,
    ROUTING_KEY_STATE,
    CtrlMsgKeys,
    CtrlMsgFields,
    RobotArmStateKeys,
    RobotMode,
)
from models.model import MotionModel
from _2D_visual.influxdb.client import InfluxClient

MEASUREMENT_MODEL = "model_prediction"
MEASUREMENT_ERROR = "tracking_error"
NUM_JOINTS = 6


class ModelTrackerService:
    """Runs the calibrated model once per motion segment, then writes
    predictions progressively as PT data arrives."""

    def __init__(self, rabbitmq_config, influxdb_config, model_params,
                 sim_dt=0.002):
        """
        Args:
            rabbitmq_config: dict passed to ``Rabbitmq(**config)``.
            influxdb_config: dict passed to ``InfluxClient(**config)``.
            model_params:    dict with keys ``vel_scale``, ``acc_scale``,
                             and optionally ``smooth_tau``.
            sim_dt:          simulation time-step for the model (seconds).
        """
        self._rmq = Rabbitmq(**rabbitmq_config)
        self._influx = InfluxClient(**influxdb_config)
        self._model = MotionModel()
        self._params = model_params
        self._sim_dt = sim_dt
        self._log = logging.getLogger(self.__class__.__name__)

        # ── last known idle joint position (updated every Idle state) ─────
        # Used as q0 when LOAD_PROGRAM arrives so we can pre-compute the
        # model immediately — before the first Running state message.
        self._last_q_idle = None
        # Fallback cmd when LOAD_PROGRAM arrives before any idle state.
        self._pending_cmd = None

        # Guard that prevents tracking stale Running states that were in-flight
        # before our LOAD_PROGRAM was sent.  Set to True on first LOAD_PROGRAM.
        self._ready_to_track = False

        # ── per-motion state (reset between motions) ──────────────────────
        self._motion_active = False
        self._motion_counter = 0      # incremented each new motion → unique series tag
        self._t_start = None          # robot TIMESTAMP of the first Running msg
        self._ts_offset_ns = None     # calibrated once: wall_ns - robot_ns
        self._prev_mode = None        # previous robot mode string

        # Interpolators built from the full model run (one per joint)
        self._q_interp = None         # position
        self._qd_interp = None        # velocity
        self._qdd_interp = None       # acceleration

        # Raw trajectory arrays for batch writing to InfluxDB
        self._t_raw = None
        self._q_raw = None
        self._qd_raw = None
        self._qdd_raw = None
        self._t_sync = None           # motion end time; batch write stops here
        self._trajectory_written = False

    # ── lifecycle ─────────────────────────────────────────────────────────

    def setup(self):
        self._rmq.connect_to_server()
        self._influx.connect()
        # Subscribe to control messages FIRST so we capture LOAD_PROGRAM
        # before the corresponding Running state arrives.
        self._rmq.subscribe(
            routing_key=ROUTING_KEY_CTRL,
            on_message_callback=self._on_ctrl_received,
        )
        self._rmq.subscribe(
            routing_key=ROUTING_KEY_STATE,
            on_message_callback=self._on_state_received,
        )
        self._log.info("Model Tracker Service ready — waiting for LOAD_PROGRAM before tracking")

    def start(self):
        self._log.info("Tracking PT motions and writing model predictions …")
        self._rmq.start_consuming()

    def stop(self):
        self._rmq.close()
        self._influx.close()

    # ── control message callback ──────────────────────────────────────────

    def _on_ctrl_received(self, ch, method, properties, body):
        """Pre-compute the model immediately on LOAD_PROGRAM using last idle q0."""
        if body.get(CtrlMsgKeys.TYPE) != CtrlMsgFields.LOAD_PROGRAM:
            return

        q_target = body[CtrlMsgKeys.JOINT_POSITIONS][0]  # first (only) waypoint
        max_vel  = float(body[CtrlMsgKeys.MAX_VELOCITY])
        acc      = float(body[CtrlMsgKeys.ACCELERATION])

        self._log.info(
            "LOAD_PROGRAM → target=%s  vel=%.1f deg/s  acc=%.1f deg/s²",
            [f"{math.degrees(v):.1f}°" for v in q_target],
            max_vel, acc,
        )

        if self._last_q_idle is not None:
            # Build interpolators now, before the first Running state arrives.
            # This eliminates the gap where PT points exist but the model line
            # has not appeared yet.
            self._run_model_with(
                q0=self._last_q_idle,
                q_target=q_target,
                max_vel=max_vel,
                acc=acc,
            )
            self._log.info("Model pre-computed at LOAD_PROGRAM time")
        else:
            # Fallback: no idle state seen yet — defer to first Running message.
            self._pending_cmd = {
                "q_target":         q_target,
                "max_velocity_deg": max_vel,
                "acceleration_deg": acc,
            }
            self._log.warning("No idle state seen yet — model will be computed on first Running message")

        self._ready_to_track = True

    # ── main callback ─────────────────────────────────────────────────────

    def _on_state_received(self, ch, method, properties, body):
        robot_mode = body[RobotArmStateKeys.ROBOT_MODE]
        q_actual = np.array(body[RobotArmStateKeys.Q_ACTUAL])
        timestamp = body[RobotArmStateKeys.TIMESTAMP]

        # Calibrate once from the first state message (during idle).
        if self._ts_offset_ns is None:
            self._ts_offset_ns = time.time_ns() - int(timestamp * 1e9)

        # Always track the last idle position so LOAD_PROGRAM can use it as q0.
        if robot_mode == RobotMode.ROBOT_MODE_IDLE:
            self._last_q_idle = list(q_actual)

        # Ignore all state messages until we have received at least one
        # LOAD_PROGRAM command this prevents false motion detection from
        # stale Running states that were in-flight when the service started.
        if not self._ready_to_track:
            self._prev_mode = robot_mode
            return

        # 1) Detect motion start  (Idle → Running)
        if not self._motion_active and robot_mode == RobotMode.ROBOT_MODE_RUNNING:
            self._motion_active = True
            self._motion_counter += 1
            self._t_start = timestamp

            # If LOAD_PROGRAM arrived before we had an idle state, the model
            # was deferred, build it now as a fallback.
            if self._q_interp is None:
                if self._pending_cmd is not None:
                    self._run_model_with(
                        q0=list(q_actual),
                        q_target=self._pending_cmd["q_target"],
                        max_vel=self._pending_cmd["max_velocity_deg"],
                        acc=self._pending_cmd["acceleration_deg"],
                    )
                    self._pending_cmd = None
                else:
                    self._run_model(body)  # last resort: read from state msg
            self._log.info("Motion started")

        # 2) Batch-write the full model trajectory the first time we see it
        if (self._motion_active and self._q_interp is not None
                and not self._trajectory_written):
            wall_t_start = self._ts_offset_ns + int(self._t_start * 1e9)
            self._write_full_trajectory(wall_t_start, self._motion_counter)

        # 3) While motion is active, write only tracking error per PT sample
        if self._motion_active and self._q_interp is not None:
            elapsed = timestamp - self._t_start
            wall_ts = self._ts_offset_ns + int(timestamp * 1e9)
            self._write_tracking_error(elapsed, q_actual, wall_ts, self._motion_counter)

        # 4) Detect motion end  (Running → Idle)
        if (self._motion_active
                and robot_mode == RobotMode.ROBOT_MODE_IDLE
                and self._prev_mode == RobotMode.ROBOT_MODE_RUNNING):
            self._motion_active = False
            self._reset_model_state()
            self._log.info("Motion ended → ready for next motion")

        self._prev_mode = robot_mode

    # ── model execution ───────────────────────────────────────────────────

    def _run_model(self, state):
        """Fallback: run the model from a state message (last resort)."""
        q0       = state[RobotArmStateKeys.Q_ACTUAL]
        q_target = state[RobotArmStateKeys.Q_TARGET]
        max_vel  = state[RobotArmStateKeys.JOINT_MAX_SPEED]
        acc      = state[RobotArmStateKeys.JOINT_MAX_ACCELERATION]
        self._log.warning("Fallback model run from state message: vel=%.1f acc=%.1f", max_vel, acc)
        self._run_model_with(q0=q0, q_target=q_target, max_vel=max_vel, acc=acc)

    def _run_model_with(self, q0, q_target, max_vel, acc):
        """Build per-joint interpolators from explicit parameters."""
        self._log.info("Model using vel=%.1f deg/s  acc=%.1f deg/s²", max_vel, acc)

        vel_scale = self._params["vel_scale"]
        acc_scale = self._params["acc_scale"]
        smooth_tau = self._params.get("smooth_tau", 0.0)

        # Estimate a generous t_end from the synchronisation time
        distances = np.abs(np.array(q_target) - np.array(q0))
        v_eff = np.deg2rad(max_vel) * np.array(vel_scale)
        a_eff = np.deg2rad(acc) * np.array(acc_scale)
        t_sync = self._model.sync_time(distances, v_eff, a_eff)
        self._t_sync = float(t_sync)  # store for batch-write trimming
        t_end = t_sync * 1.5 + 1.0

        t_mod, q_mod, qd_mod, qdd_mod = self._model.simulate_joint_motion(
            q0=q0,
            q_target=q_target,
            max_velocity_deg=max_vel,
            acceleration_deg=acc,
            dt=self._sim_dt,
            t_end=t_end,
            vel_scale=vel_scale,
            acc_scale=acc_scale,
            smooth_tau=smooth_tau,
        )

        # Store raw arrays for the batch write at motion start
        self._t_raw = t_mod
        self._q_raw = q_mod
        self._qd_raw = qd_mod
        self._qdd_raw = qdd_mod
        self._trajectory_written = False

        # Build linear interpolators so we can sample at arbitrary elapsed times
        self._q_interp = [
            interp1d(t_mod, q_mod[:, j], kind="linear",
                     fill_value="extrapolate")
            for j in range(NUM_JOINTS)
        ]
        self._qd_interp = [
            interp1d(t_mod, qd_mod[:, j], kind="linear",
                     fill_value="extrapolate")
            for j in range(NUM_JOINTS)
        ]
        self._qdd_interp = [
            interp1d(t_mod, qdd_mod[:, j], kind="linear",
                     fill_value="extrapolate")
            for j in range(NUM_JOINTS)
        ]

    # ── writing ────────────────────────────────────────────────────────────

    def _write_full_trajectory(self, wall_t_start_ns, motion_id):
        """Batch-write the model trajectory (up to t_sync) so the red line is
        instantly available in InfluxDB.  Grafana's ``range(stop: now())``
        reveals each point as wall-clock time reaches the timestamp.
        The trajectory is trimmed at t_sync to avoid a flat settling tail."""
        t_cutoff = self._t_sync + 0.1   # small buffer past the sync point
        step = max(1, int(0.01 / self._sim_dt))   # write ~every 10 ms
        records = []
        for i in range(0, len(self._t_raw), step):
            if self._t_raw[i] > t_cutoff:
                break
            ts_ns = wall_t_start_ns + int(self._t_raw[i] * 1e9)
            p = (Point(MEASUREMENT_MODEL)
                 .tag("source", "model"))
            for j in range(NUM_JOINTS):
                p = p.field(f"q_j{j}", float(np.rad2deg(self._q_raw[i, j])))
                p = p.field(f"qd_j{j}", float(np.rad2deg(self._qd_raw[i, j])))
                p = p.field(f"qdd_j{j}", float(np.rad2deg(self._qdd_raw[i, j])))
            p = p.time(ts_ns, WritePrecision.NS)
            records.append(p)
        self._influx.write_points(records)
        self._trajectory_written = True
        self._log.info("Batch-wrote %d model points for motion %d (t_sync=%.2fs)",
                       len(records), motion_id, self._t_sync)

    def _write_tracking_error(self, elapsed, q_actual, wall_ts, motion_id):
        """Write only the tracking error (model vs observed) for this PT sample."""
        elapsed = max(0.0, elapsed)
        tags = {"source": "model", "motion_id": str(motion_id)}
        error_fields = {}
        for j in range(NUM_JOINTS):
            q_pred = float(self._q_interp[j](elapsed))
            error_fields[f"err_j{j}"] = np.rad2deg(q_pred - q_actual[j])
        self._influx.write_point(
            measurement=MEASUREMENT_ERROR,
            fields=error_fields,
            tags=tags,
            timestamp_ns=wall_ts,
        )

    # ── cleanup ───────────────────────────────────────────────────────────

    def _reset_model_state(self):
        self._t_start = None
        self._q_interp = None
        self._qd_interp = None
        self._qdd_interp = None
        self._t_raw = None
        self._q_raw = None
        self._qd_raw = None
        self._qdd_raw = None
        self._t_sync = None
        self._trajectory_written = False
