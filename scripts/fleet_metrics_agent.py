#!/usr/bin/env python3
# Klipper fleet KPI upload agent.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import argparse
import configparser
import json
import logging
import os
import socket
import sqlite3
import sys
import time
import uuid
from urllib import error as urlerror
from urllib import request as urlrequest


FRAME_TERMINATOR = b"\x03"
DEFAULT_CONFIG = "/etc/klipper/fleet_metrics_agent.conf"
DEFAULT_SOCKET = "/tmp/klippy_uds"
DEFAULT_QUEUE_DB = "/var/lib/klipper-fleet-agent/queue.sqlite3"
DEFAULT_HEARTBEAT_INTERVAL = 60.0
DEFAULT_UPLOAD_BATCH_SIZE = 100


def utc_now():
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def encode_frame(message):
    return json.dumps(message, separators=(",", ":"), sort_keys=True).encode(
        "utf-8") + FRAME_TERMINATOR


class KlipperFrameDecoder:
    def __init__(self):
        self.partial = b""

    def feed(self, data):
        data = self.partial + data
        parts = data.split(FRAME_TERMINATOR)
        self.partial = parts.pop()
        messages = []
        for part in parts:
            if not part:
                continue
            messages.append(json.loads(part.decode("utf-8")))
        return messages


class KlipperAPIClient:
    def __init__(self, socket_path, timeout=5.0):
        self.socket_path = socket_path
        self.timeout = timeout
        self.sock = None
        self.decoder = KlipperFrameDecoder()
        self.next_id = 1

    def connect(self):
        self.close()
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect(self.socket_path)
        self.decoder = KlipperFrameDecoder()

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except socket.error:
                pass
        self.sock = None

    def send_request(self, method, params=None):
        req_id = self.next_id
        self.next_id += 1
        message = {"id": req_id, "method": method}
        if params is not None:
            message["params"] = params
        self.sock.sendall(encode_frame(message))
        return req_id

    def recv_messages(self):
        data = self.sock.recv(65536)
        if not data:
            raise socket.error("Klipper API socket closed")
        return self.decoder.feed(data)


class FleetConfig:
    def __init__(self, printer_id, backend_url, auth_token,
                 klipper_socket=DEFAULT_SOCKET,
                 queue_db=DEFAULT_QUEUE_DB,
                 heartbeat_interval=DEFAULT_HEARTBEAT_INTERVAL,
                 upload_batch_size=DEFAULT_UPLOAD_BATCH_SIZE,
                 request_timeout=10.0):
        self.printer_id = printer_id
        self.backend_url = backend_url.rstrip("/")
        self.auth_token = auth_token
        self.klipper_socket = klipper_socket
        self.queue_db = queue_db
        self.heartbeat_interval = float(heartbeat_interval)
        self.upload_batch_size = int(upload_batch_size)
        self.request_timeout = float(request_timeout)

    @classmethod
    def from_file(cls, filename):
        parser = configparser.ConfigParser()
        read_files = parser.read(filename)
        if not read_files:
            raise SystemExit("Unable to read config file '%s'" % (filename,))
        if not parser.has_section("fleet"):
            raise SystemExit("Missing [fleet] section in '%s'" % (filename,))
        sec = parser["fleet"]
        missing = [
            key for key in ("printer_id", "backend_url", "auth_token")
            if not sec.get(key)
        ]
        if missing:
            raise SystemExit("Missing fleet config keys: %s"
                             % (", ".join(missing),))
        return cls(
            printer_id=sec.get("printer_id"),
            backend_url=sec.get("backend_url"),
            auth_token=sec.get("auth_token"),
            klipper_socket=sec.get("klipper_socket", DEFAULT_SOCKET),
            queue_db=sec.get("queue_db", DEFAULT_QUEUE_DB),
            heartbeat_interval=sec.getfloat(
                "heartbeat_interval_seconds", DEFAULT_HEARTBEAT_INTERVAL),
            upload_batch_size=sec.getint(
                "upload_batch_size", DEFAULT_UPLOAD_BATCH_SIZE),
            request_timeout=sec.getfloat("request_timeout_seconds", 10.0))


class EventQueue:
    def __init__(self, filename):
        self.filename = filename
        dirname = os.path.dirname(filename)
        if dirname and not os.path.exists(dirname):
            os.makedirs(dirname)
        self.conn = sqlite3.connect(filename)
        self.conn.row_factory = sqlite3.Row
        self._init_schema()

    def _init_schema(self):
        self.conn.execute("""
            CREATE TABLE IF NOT EXISTS outbound_events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                event_id TEXT NOT NULL UNIQUE,
                kind TEXT NOT NULL,
                payload_json TEXT NOT NULL,
                created_at REAL NOT NULL,
                attempts INTEGER NOT NULL DEFAULT 0,
                last_error TEXT
            )
        """)
        self.conn.execute("""
            CREATE INDEX IF NOT EXISTS outbound_events_created_idx
            ON outbound_events(created_at, id)
        """)
        self.conn.commit()

    def enqueue(self, event):
        payload = dict(event)
        event_id = payload.setdefault("event_id", str(uuid.uuid4()))
        kind = payload.get("kind", "event")
        self.conn.execute("""
            INSERT OR IGNORE INTO outbound_events
                (event_id, kind, payload_json, created_at)
            VALUES (?, ?, ?, ?)
        """, (event_id, kind, json.dumps(payload, sort_keys=True),
              time.time()))
        self.conn.commit()
        return event_id

    def pending_batch(self, limit):
        cur = self.conn.execute("""
            SELECT id, payload_json FROM outbound_events
            ORDER BY created_at, id LIMIT ?
        """, (int(limit),))
        rows = cur.fetchall()
        return [(row["id"], json.loads(row["payload_json"])) for row in rows]

    def mark_sent(self, row_ids):
        if not row_ids:
            return
        self.conn.executemany(
            "DELETE FROM outbound_events WHERE id = ?",
            [(row_id,) for row_id in row_ids])
        self.conn.commit()

    def mark_failed(self, row_ids, message):
        if not row_ids:
            return
        trimmed = str(message)[:500]
        self.conn.executemany("""
            UPDATE outbound_events
            SET attempts = attempts + 1, last_error = ?
            WHERE id = ?
        """, [(trimmed, row_id) for row_id in row_ids])
        self.conn.commit()

    def count(self):
        cur = self.conn.execute("SELECT COUNT(*) FROM outbound_events")
        return int(cur.fetchone()[0])

    def close(self):
        self.conn.close()


class PrintLifecycleTracker:
    TERMINAL_STATES = set(["complete", "error", "cancelled"])

    def __init__(self, printer_id):
        self.printer_id = printer_id
        self.active_job_id = None
        self.active_filename = ""
        self.last_state = None

    def update(self, print_stats, eventtime=None):
        if not print_stats:
            return []
        state = print_stats.get("state")
        filename = print_stats.get("filename") or ""
        events = []
        if state == "printing":
            if (self.active_job_id is None
                    or (filename and filename != self.active_filename
                        and self.last_state in self.TERMINAL_STATES)):
                self.active_job_id = str(uuid.uuid4())
                self.active_filename = filename
                events.append(self._event(
                    "print_started", print_stats, eventtime,
                    state="printing"))
        elif state in self.TERMINAL_STATES:
            if self.last_state != state or self.active_job_id is not None:
                if self.active_job_id is None:
                    self.active_job_id = str(uuid.uuid4())
                    self.active_filename = filename
                events.append(self._event(
                    "print_finished", print_stats, eventtime, state=state))
                self.active_job_id = None
                self.active_filename = ""
        self.last_state = state
        return events

    def _event(self, kind, print_stats, eventtime, state):
        info = print_stats.get("info") or {}
        return {
            "event_id": str(uuid.uuid4()),
            "kind": kind,
            "printer_id": self.printer_id,
            "job_id": self.active_job_id,
            "observed_at": utc_now(),
            "eventtime": eventtime,
            "state": state,
            "filename": print_stats.get("filename") or "",
            "total_duration": float(print_stats.get("total_duration") or 0.0),
            "print_duration": float(print_stats.get("print_duration") or 0.0),
            "filament_used": float(print_stats.get("filament_used") or 0.0),
            "message": print_stats.get("message") or "",
            "total_layer": info.get("total_layer"),
            "current_layer": info.get("current_layer"),
        }


class StateEventTracker:
    def __init__(self, printer_id):
        self.printer_id = printer_id
        self.last_webhooks_state = None

    def update(self, status, eventtime=None):
        webhooks = status.get("webhooks") or {}
        state = webhooks.get("state")
        events = []
        if state in ("shutdown", "error") and state != self.last_webhooks_state:
            events.append({
                "event_id": str(uuid.uuid4()),
                "kind": "printer_state",
                "printer_id": self.printer_id,
                "observed_at": utc_now(),
                "eventtime": eventtime,
                "state": state,
                "message": webhooks.get("state_message") or "",
            })
        self.last_webhooks_state = state
        return events


def current_print_payload(status):
    ps = status.get("print_stats") or {}
    vsd = status.get("virtual_sdcard") or {}
    return {
        "filename": ps.get("filename") or "",
        "state": ps.get("state"),
        "message": ps.get("message") or "",
        "total_duration": ps.get("total_duration"),
        "print_duration": ps.get("print_duration"),
        "filament_used": ps.get("filament_used"),
        "progress": vsd.get("progress"),
        "file_position": vsd.get("file_position"),
        "file_size": vsd.get("file_size"),
    }


def mcu_payload(status):
    result = {}
    for name, value in status.items():
        if name == "mcu" or name.startswith("mcu "):
            result[name] = {
                "mcu_version": value.get("mcu_version"),
                "mcu_build_versions": value.get("mcu_build_versions"),
                "last_stats": value.get("last_stats"),
            }
    return result


def build_heartbeat(config, info, status, queue_depth):
    webhooks = status.get("webhooks") or {}
    system_stats = status.get("system_stats") or {}
    return {
        "printer_id": config.printer_id,
        "observed_at": utc_now(),
        "state": webhooks.get("state"),
        "state_message": webhooks.get("state_message"),
        "hostname": info.get("hostname"),
        "software_version": info.get("software_version"),
        "config_file": info.get("config_file"),
        "cpu_info": info.get("cpu_info"),
        "klipper_path": info.get("klipper_path"),
        "python_path": info.get("python_path"),
        "system_stats": system_stats,
        "mcu": mcu_payload(status),
        "current_print": current_print_payload(status),
        "queue_depth": queue_depth,
    }


class Uploader:
    def __init__(self, config):
        self.config = config

    def post_json(self, path, payload):
        body = json.dumps(payload, sort_keys=True).encode("utf-8")
        req = urlrequest.Request(
            self.config.backend_url + path,
            data=body,
            headers={
                "Authorization": "Bearer " + self.config.auth_token,
                "Content-Type": "application/json",
                "User-Agent": "klipper-fleet-metrics-agent/1",
            },
            method="POST")
        with urlrequest.urlopen(
                req, timeout=self.config.request_timeout) as res:
            status = getattr(res, "status", res.getcode())
            if status < 200 or status >= 300:
                raise IOError("HTTP %s from %s" % (status, path))
            data = res.read()
            if not data:
                return {}
            return json.loads(data.decode("utf-8"))

    def upload_heartbeat(self, heartbeat):
        return self.post_json("/api/v1/heartbeat", heartbeat)

    def upload_events(self, printer_id, events):
        return self.post_json("/api/v1/events/batch", {
            "printer_id": printer_id,
            "events": events,
        })


class FleetMetricsAgent:
    SUBSCRIPTION_OBJECTS = {
        "webhooks": None,
        "print_stats": None,
        "virtual_sdcard": None,
        "mcu": ["mcu_version", "mcu_build_versions", "last_stats"],
        "system_stats": None,
    }

    def __init__(self, config, queue=None, uploader=None):
        self.config = config
        self.queue = queue or EventQueue(config.queue_db)
        self.uploader = uploader or Uploader(config)
        self.api = KlipperAPIClient(config.klipper_socket)
        self.info = {}
        self.status = {}
        self.print_tracker = PrintLifecycleTracker(config.printer_id)
        self.state_tracker = StateEventTracker(config.printer_id)
        self.last_heartbeat_time = 0.0

    def run_forever(self):
        while True:
            try:
                self._run_connected()
            except KeyboardInterrupt:
                raise
            except Exception as e:
                logging.warning("fleet agent reconnecting after error: %s", e)
                self.api.close()
                time.sleep(5.0)

    def _run_connected(self):
        self.api.connect()
        self.api.send_request("info", {
            "client_info": {
                "name": "klipper-fleet-metrics-agent",
                "version": "1",
            }
        })
        self.api.send_request("objects/subscribe", {
            "objects": self.SUBSCRIPTION_OBJECTS,
            "response_template": {"fleet_metrics": "status_update"},
        })
        while True:
            now = time.monotonic()
            if now - self.last_heartbeat_time >= self.config.heartbeat_interval:
                self.flush_once()
                self.last_heartbeat_time = now
            try:
                messages = self.api.recv_messages()
            except socket.timeout:
                continue
            for message in messages:
                self.handle_klipper_message(message)

    def handle_klipper_message(self, message):
        if "error" in message:
            logging.warning("Klipper API error: %s", message["error"])
            return
        params = None
        if "result" in message:
            result = message["result"]
            if "software_version" in result or "hostname" in result:
                self.info.update(result)
            params = result
        elif "params" in message:
            params = message["params"]
        if not params:
            return
        status = params.get("status")
        eventtime = params.get("eventtime")
        if not status:
            return
        for name, values in status.items():
            if isinstance(values, dict):
                current = self.status.setdefault(name, {})
                current.update(values)
            else:
                self.status[name] = values
        events = []
        events.extend(self.print_tracker.update(
            self.status.get("print_stats"), eventtime))
        events.extend(self.state_tracker.update(self.status, eventtime))
        for event in events:
            self.queue.enqueue(event)

    def flush_once(self):
        if self.info or self.status:
            heartbeat = build_heartbeat(
                self.config, self.info, self.status, self.queue.count())
            self.uploader.upload_heartbeat(heartbeat)
        batch = self.queue.pending_batch(self.config.upload_batch_size)
        if not batch:
            return
        row_ids = [row_id for row_id, event in batch]
        events = [event for row_id, event in batch]
        try:
            self.uploader.upload_events(self.config.printer_id, events)
        except (IOError, OSError, urlerror.URLError, ValueError) as e:
            self.queue.mark_failed(row_ids, e)
            raise
        self.queue.mark_sent(row_ids)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Upload Klipper printer KPI events to a fleet backend.")
    parser.add_argument(
        "-c", "--config", default=DEFAULT_CONFIG,
        help="Path to fleet agent config file.")
    parser.add_argument("--printer-id", help="Override configured printer_id.")
    parser.add_argument(
        "--backend-url", help="Override configured backend_url.")
    parser.add_argument("--auth-token", help="Override configured auth_token.")
    parser.add_argument(
        "--klipper-socket", help="Override configured Klipper API socket.")
    parser.add_argument("--queue-db", help="Override configured SQLite queue.")
    parser.add_argument(
        "--once", action="store_true",
        help="Connect long enough to receive initial status and upload once.")
    parser.add_argument(
        "-v", "--verbose", action="store_true",
        help="Enable verbose logging.")
    return parser.parse_args(argv)


def apply_overrides(config, args):
    for attr, value in [
            ("printer_id", args.printer_id),
            ("backend_url", args.backend_url),
            ("auth_token", args.auth_token),
            ("klipper_socket", args.klipper_socket),
            ("queue_db", args.queue_db)]:
        if value:
            setattr(config, attr, value.rstrip("/")
                    if attr == "backend_url" else value)
    return config


def main(argv=None):
    args = parse_args(argv or sys.argv[1:])
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s")
    config = apply_overrides(FleetConfig.from_file(args.config), args)
    agent = FleetMetricsAgent(config)
    if args.once:
        agent.api.connect()
        agent.api.send_request("info")
        agent.api.send_request("objects/query", {
            "objects": FleetMetricsAgent.SUBSCRIPTION_OBJECTS,
        })
        deadline = time.monotonic() + 10.0
        while (time.monotonic() < deadline
               and (not agent.info or not agent.status)):
            for message in agent.api.recv_messages():
                agent.handle_klipper_message(message)
        agent.flush_once()
        return
    agent.run_forever()


if __name__ == "__main__":
    main()
