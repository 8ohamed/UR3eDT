"""UR3e Digital Twin Manager (entry point).

Starts the DT services and keeps them running:
    • PTRecorderService: records PT joint data to InfluxDB (only during motion)
    • ModelTrackerService: runs the calibrated model on each motion, writes
                           predictions + tracking error to InfluxDB

Prerequisites:
    • RabbitMQ running  (docker compose -f communication/installation/docker-compose.yml up -d)
    • InfluxDB running  (docker compose -f _2D_visual/docker-compose.yml up -d)
    • PT mockup running (python -m startup.start_ur3e_mockup)

Run from the UR3eDT root:
    python dt_manager.py

Then trigger motions separately:
    python mini_controller.py --target 30 45 -30 10 20 -60
"""

import argparse
import logging
import threading
import time

from _2D_visual.pt_recorder_service import PTRecorderService
from _2D_visual.model_tracker_service import ModelTrackerService
from calibration.utils import load_parameters


RABBITMQ_CONFIG = {
    "ip":       "localhost",
    "port":     5672,
    "username": "ur3e",
    "password": "ur3e",
    "vhost":    "/",
    "exchange": "UR3E_AMQP",
    "type":     "topic",
}

INFLUXDB_CONFIG = {
    "url":    "http://localhost:8086",
    "token":  "ur3e-dt-token",
    "org":    "ur3e_dt",
    "bucket": "ur3e",
}

MODEL_PARAMS_PATH = "./models/parameters.json"


def _start_service_thread(service, name):
    """Setup and start a service in a daemon thread."""
    service.setup()
    t = threading.Thread(target=service.start, daemon=True, name=name)
    t.start()
    return t


# ── main ───────────────────────────────────────────────────────────────────────

def _parse_args():
    p = argparse.ArgumentParser(description="UR3e DT Manager")
    return p.parse_args()


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s  [%(name)-20s]  %(levelname)s: %(message)s",
    )
    logging.getLogger("pika").setLevel(logging.WARNING)
    log = logging.getLogger("DTManager")
    _parse_args()

    # ── 1. load calibrated model parameters ───────────────────────────────
    model_params = load_parameters(MODEL_PARAMS_PATH)

    # ── 2. start services ─────────────────────────────────────────────────
    log.info("Starting PT recorder service …")
    recorder = PTRecorderService(dict(RABBITMQ_CONFIG), INFLUXDB_CONFIG)
    _start_service_thread(recorder, "PTRecorder")

    log.info("Starting model tracker service …")
    tracker = ModelTrackerService(
        dict(RABBITMQ_CONFIG), INFLUXDB_CONFIG, model_params
    )
    _start_service_thread(tracker, "ModelTracker")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        log.info("Shutting down …")
        recorder.stop()
        tracker.stop()
        log.info("Done.")


if __name__ == "__main__":
    main()
