"""Thin wrapper around the InfluxDB 2.x Python client for the UR3e Digital Twin."""

import time
import logging
import urllib.request
import urllib.error

from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS


class InfluxClient:
    """Connect to InfluxDB 2.x, write points, and run Flux queries."""

    def __init__(self, url, token, org, bucket):
        self.url = url
        self.token = token
        self.org = org
        self.bucket = bucket
        self._client = None
        self._write_api = None
        self._query_api = None
        self._log = logging.getLogger(self.__class__.__name__)

    # ── lifecycle ─────────────────────────────────────────────────────────

    def _wait_until_ready(self, timeout=60, interval=2):
        """Poll /health until InfluxDB reports status=pass, or raise on timeout."""
        health_url = self.url.rstrip("/") + "/health"
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                with urllib.request.urlopen(health_url, timeout=3) as resp:
                    if resp.status == 200:
                        return
            except Exception:
                pass
            self._log.info("Waiting for InfluxDB to be ready …")
            time.sleep(interval)
        raise TimeoutError(f"InfluxDB at {self.url} did not become ready within {timeout}s")

    def connect(self):
        self._wait_until_ready()
        self._client = InfluxDBClient(url=self.url, token=self.token, org=self.org)
        self._write_api = self._client.write_api(write_options=SYNCHRONOUS)
        self._query_api = self._client.query_api()
        self._log.info("Connected to InfluxDB at %s", self.url)

    def close(self):
        if self._client:
            self._client.close()
            self._log.info("InfluxDB connection closed")

    # ── writing ───────────────────────────────────────────────────────────

    def write_point(self, measurement, fields, tags=None, timestamp_ns=None):
        """Write a single point to the configured bucket.

        Args:
            measurement: InfluxDB measurement name.
            fields:      dict of field_name → numeric/string value.
            tags:        optional dict of tag_name → tag_value.
            timestamp_ns: optional epoch nanoseconds; defaults to now.
        """
        point = Point(measurement)
        if tags:
            for k, v in tags.items():
                point = point.tag(k, v)
        for k, v in fields.items():
            point = point.field(k, float(v) if isinstance(v, (int, float)) else v)
        point = point.time(timestamp_ns or time.time_ns(), WritePrecision.NS)
        self._write_api.write(bucket=self.bucket, org=self.org, record=point)

    def write_points(self, records):
        """Write a batch of dict-format records to the configured bucket.

        Each record: {"measurement": ..., "tags": {...}, "fields": {...}, "time": ns}
        """
        self._write_api.write(bucket=self.bucket, org=self.org, record=records)

    # ── querying ──────────────────────────────────────────────────────────

    def query(self, flux_query):
        """Run a Flux query and return the result tables."""
        return self._query_api.query(org=self.org, query=flux_query)

    def query_dataframe(self, flux_query):
        """Run a Flux query and return a pandas DataFrame."""
        return self._query_api.query_data_frame(org=self.org, query=flux_query)
