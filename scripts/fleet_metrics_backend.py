#!/usr/bin/env python3
# Minimal HTTP ingestion backend for Klipper fleet KPI uploads.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import argparse
import json
import logging
import os
import sys
from http import server


DEFAULT_LISTEN_HOST = "0.0.0.0"
DEFAULT_LISTEN_PORT = 8080


def _load_psycopg():
    try:
        import psycopg
        return "psycopg3", psycopg
    except ImportError:
        pass
    try:
        import psycopg2
        return "psycopg2", psycopg2
    except ImportError:
        pass
    raise SystemExit(
        "Install psycopg or psycopg2 to use the PostgreSQL backend")


class PostgresStore:
    def __init__(self, database_url):
        self.database_url = database_url
        self.driver_name, self.driver = _load_psycopg()

    def _connect(self):
        return self.driver.connect(self.database_url)

    def ingest_heartbeat(self, payload):
        with self._connect() as conn:
            with conn.cursor() as cur:
                cur.execute(
                    "SELECT ingest_fleet_heartbeat(%s::jsonb)",
                    (json.dumps(payload),))
            conn.commit()

    def ingest_events(self, events):
        with self._connect() as conn:
            with conn.cursor() as cur:
                for event in events:
                    cur.execute(
                        "SELECT ingest_fleet_event(%s::jsonb)",
                        (json.dumps(event),))
            conn.commit()


class FleetBackendHandler(server.BaseHTTPRequestHandler):
    server_version = "KlipperFleetKPI/1"

    def log_message(self, fmt, *args):
        logging.info("%s - %s", self.address_string(), fmt % args)

    def do_POST(self):
        try:
            self._handle_post()
        except ValueError as e:
            self._send_json(400, {"error": str(e)})
        except PermissionError as e:
            self._send_json(401, {"error": str(e)})
        except Exception as e:
            logging.exception("request failed")
            self._send_json(500, {"error": str(e)})

    def _handle_post(self):
        self._check_auth()
        payload = self._read_json()
        if self.path == "/api/v1/heartbeat":
            self.server.store.ingest_heartbeat(payload)
            self._send_json(200, {"ok": True})
            return
        if self.path == "/api/v1/events/batch":
            events = payload.get("events")
            if not isinstance(events, list):
                raise ValueError("events must be a list")
            self.server.store.ingest_events(events)
            self._send_json(200, {"ok": True, "accepted": len(events)})
            return
        self._send_json(404, {"error": "not found"})

    def _check_auth(self):
        header = self.headers.get("Authorization", "")
        prefix = "Bearer "
        if not header.startswith(prefix):
            raise PermissionError("missing bearer token")
        token = header[len(prefix):]
        if token not in self.server.auth_tokens:
            raise PermissionError("invalid bearer token")

    def _read_json(self):
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            raise ValueError("invalid content length")
        if length <= 0:
            raise ValueError("empty request body")
        raw = self.rfile.read(length)
        try:
            return json.loads(raw.decode("utf-8"))
        except ValueError:
            raise ValueError("invalid json body")

    def _send_json(self, status, payload):
        body = json.dumps(payload, sort_keys=True).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class FleetHTTPServer(server.ThreadingHTTPServer):
    def __init__(self, address, store, auth_tokens):
        super().__init__(address, FleetBackendHandler)
        self.store = store
        self.auth_tokens = set(auth_tokens)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Receive Klipper fleet KPI uploads into PostgreSQL.")
    parser.add_argument(
        "--database-url", default=os.environ.get("DATABASE_URL"),
        help="PostgreSQL connection URL.")
    parser.add_argument(
        "--auth-token", action="append",
        default=None,
        help="Accepted bearer token. May be specified multiple times.")
    parser.add_argument(
        "--listen-host",
        default=os.environ.get("LISTEN_HOST", DEFAULT_LISTEN_HOST))
    parser.add_argument(
        "--listen-port", type=int,
        default=int(os.environ.get("LISTEN_PORT", DEFAULT_LISTEN_PORT)))
    parser.add_argument(
        "-v", "--verbose", action="store_true",
        help="Enable verbose logging.")
    return parser.parse_args(argv)


def tokens_from_env(args):
    tokens = []
    env_token = os.environ.get("FLEET_KPI_TOKEN", "")
    if env_token:
        tokens.extend([t.strip() for t in env_token.split(",") if t.strip()])
    if args.auth_token:
        tokens.extend(args.auth_token)
    if not tokens:
        raise SystemExit("Configure FLEET_KPI_TOKEN or --auth-token")
    return tokens


def main(argv=None):
    args = parse_args(argv or sys.argv[1:])
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s")
    if not args.database_url:
        raise SystemExit("Configure DATABASE_URL or --database-url")
    httpd = FleetHTTPServer(
        (args.listen_host, args.listen_port),
        PostgresStore(args.database_url),
        tokens_from_env(args))
    logging.info("fleet backend listening on %s:%d",
                 args.listen_host, args.listen_port)
    httpd.serve_forever()


if __name__ == "__main__":
    main()
