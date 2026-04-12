"""Service that subscribes to PT state messages and writes raw data to InfluxDB.

Measurement written: ``pt_state``
    Fields per joint (j = 0..5):
        q_j0 … q_j5: actual joint position  (degrees)
        qd_j0 … qd_j5: actual joint velocity  (deg/s)
    Extra fields:
        robot_mode: "Running" / "Idle"
"""

import logging
import time
import numpy as np

from communication.rabbitmq import Rabbitmq
from communication.protocol import ROUTING_KEY_STATE, RobotArmStateKeys, RobotMode
from _2D_visual.influxdb.client import InfluxClient

MEASUREMENT_PT = "pt_state"
NUM_JOINTS = 6


class PTRecorderService:
    """Records PT state to InfluxDB only while the robot is in motion.

    Writing starts on the first Running state and stops when the robot
    returns to Idle — matching the window the model tracker uses.
    """

    def __init__(self, rabbitmq_config, influxdb_config):
        self._rmq = Rabbitmq(**rabbitmq_config)
        self._influx = InfluxClient(**influxdb_config)
        self._log = logging.getLogger(self.__class__.__name__)
        self._motion_active = False
        self._ts_offset_ns = None   # calibrated on first state msg: wall_ns - robot_ns
        self._prev_mode = None

    # ── lifecycle ─────────────────────────────────────────────────────────

    def setup(self):
        self._rmq.connect_to_server()
        self._influx.connect()
        self._rmq.subscribe(
            routing_key=ROUTING_KEY_STATE,
            on_message_callback=self._on_state_received,
        )
        self._log.info("PT Recorder Service ready")

    def start(self):
        self._log.info("Recording PT data → InfluxDB …")
        self._rmq.start_consuming()

    def stop(self):
        self._rmq.close()
        self._influx.close()

    # ── callback ──────────────────────────────────────────────────────────

    def _on_state_received(self, ch, method, properties, body):
        robot_mode = body[RobotArmStateKeys.ROBOT_MODE]
        robot_timestamp = body[RobotArmStateKeys.TIMESTAMP]

        # Calibrate once from the first message received (during idle).
        # Both services do this independently; because robot-clock and
        # wall-clock both measure real seconds the offset is stable, so
        # both services derive the same value regardless of which message
        # they happen to calibrate on.
        if self._ts_offset_ns is None:
            self._ts_offset_ns = time.time_ns() - int(robot_timestamp * 1e9)

        # Start recording on Idle → Running transition
        if not self._motion_active and robot_mode == RobotMode.ROBOT_MODE_RUNNING:
            self._motion_active = True
            self._log.info("Motion started → recording PT data")

        # Stop recording on Running → Idle transition
        if (self._motion_active
                and robot_mode == RobotMode.ROBOT_MODE_IDLE
                and self._prev_mode == RobotMode.ROBOT_MODE_RUNNING):
            self._motion_active = False
            self._log.info("Motion ended → stopped recording PT data")

        self._prev_mode = robot_mode

        if not self._motion_active:
            return

        q_actual = body[RobotArmStateKeys.Q_ACTUAL]
        qd_actual = body.get(RobotArmStateKeys.QD_ACTUAL, [0.0] * NUM_JOINTS)

        fields = {"robot_mode": robot_mode}
        for j in range(NUM_JOINTS):
            fields[f"q_j{j}"] = float(np.rad2deg(q_actual[j]))
            fields[f"qd_j{j}"] = float(np.rad2deg(qd_actual[j]))

        timestamp_ns = self._ts_offset_ns + int(robot_timestamp * 1e9)

        self._influx.write_point(
            measurement=MEASUREMENT_PT,
            fields=fields,
            tags={"source": "physical_twin"},
            timestamp_ns=timestamp_ns,
        )
