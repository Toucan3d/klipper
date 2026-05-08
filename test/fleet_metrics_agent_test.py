import importlib.util
import json
import os
import tempfile
import threading
import unittest
from urllib import error as urlerror
from urllib import request as urlrequest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AGENT_PATH = os.path.join(ROOT, "scripts", "fleet_metrics_agent.py")
spec = importlib.util.spec_from_file_location("fleet_metrics_agent", AGENT_PATH)
fleet = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fleet)
BACKEND_PATH = os.path.join(ROOT, "scripts", "fleet_metrics_backend.py")
backend_spec = importlib.util.spec_from_file_location(
    "fleet_metrics_backend", BACKEND_PATH)
backend = importlib.util.module_from_spec(backend_spec)
backend_spec.loader.exec_module(backend)


class FrameDecoderTest(unittest.TestCase):
    def test_decodes_split_frames(self):
        decoder = fleet.KlipperFrameDecoder()
        frame1 = fleet.encode_frame({"id": 1, "result": {"state": "ready"}})
        frame2 = fleet.encode_frame({"params": {"eventtime": 12.0}})
        self.assertEqual(decoder.feed(frame1[:5]), [])
        messages = decoder.feed(frame1[5:] + frame2)
        self.assertEqual(messages[0]["id"], 1)
        self.assertEqual(messages[1]["params"]["eventtime"], 12.0)


class EventQueueTest(unittest.TestCase):
    def test_enqueue_mark_failed_and_mark_sent(self):
        with tempfile.TemporaryDirectory() as td:
            queue = fleet.EventQueue(os.path.join(td, "queue.sqlite3"))
            queue.enqueue({
                "event_id": "00000000-0000-0000-0000-000000000001",
                "kind": "print_started",
                "printer_id": "p1",
            })
            queue.enqueue({
                "event_id": "00000000-0000-0000-0000-000000000001",
                "kind": "print_started",
                "printer_id": "p1",
            })
            self.assertEqual(queue.count(), 1)
            batch = queue.pending_batch(10)
            self.assertEqual(len(batch), 1)
            queue.mark_failed([batch[0][0]], "network down")
            self.assertEqual(queue.count(), 1)
            queue.mark_sent([batch[0][0]])
            self.assertEqual(queue.count(), 0)
            queue.close()


class PrintLifecycleTrackerTest(unittest.TestCase):
    def test_print_start_and_complete_events(self):
        tracker = fleet.PrintLifecycleTracker("printer-a")
        start_events = tracker.update({
            "state": "printing",
            "filename": "part.gcode",
            "total_duration": 1.0,
            "print_duration": 0.5,
            "filament_used": 0.0,
            "info": {"total_layer": 5, "current_layer": 1},
        }, eventtime=10.0)
        self.assertEqual(len(start_events), 1)
        self.assertEqual(start_events[0]["kind"], "print_started")
        job_id = start_events[0]["job_id"]

        repeat_events = tracker.update({
            "state": "printing",
            "filename": "part.gcode",
        }, eventtime=11.0)
        self.assertEqual(repeat_events, [])

        finish_events = tracker.update({
            "state": "complete",
            "filename": "part.gcode",
            "total_duration": 100.0,
            "print_duration": 90.0,
            "filament_used": 1234.0,
            "message": "",
            "info": {"total_layer": 5, "current_layer": 5},
        }, eventtime=110.0)
        self.assertEqual(len(finish_events), 1)
        self.assertEqual(finish_events[0]["kind"], "print_finished")
        self.assertEqual(finish_events[0]["state"], "complete")
        self.assertEqual(finish_events[0]["job_id"], job_id)

    def test_error_without_seen_start_still_records_terminal_event(self):
        tracker = fleet.PrintLifecycleTracker("printer-a")
        events = tracker.update({
            "state": "error",
            "filename": "failed.gcode",
            "message": "Move out of range",
        })
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["state"], "error")
        self.assertTrue(events[0]["job_id"])


class HeartbeatTest(unittest.TestCase):
    def test_builds_heartbeat_from_status(self):
        config = fleet.FleetConfig(
            printer_id="printer-a",
            backend_url="http://pi:8080",
            auth_token="token")
        heartbeat = fleet.build_heartbeat(config, {
            "hostname": "host-a",
            "software_version": "v1",
            "config_file": "/home/pi/printer.cfg",
        }, {
            "webhooks": {"state": "ready", "state_message": "Printer is ready"},
            "system_stats": {"sysload": 0.1},
            "mcu": {"mcu_version": "fw1", "mcu_build_versions": "gcc"},
            "print_stats": {"state": "printing", "filename": "x.gcode"},
            "virtual_sdcard": {"progress": 0.5},
        }, queue_depth=3)
        self.assertEqual(heartbeat["printer_id"], "printer-a")
        self.assertEqual(heartbeat["state"], "ready")
        self.assertEqual(heartbeat["mcu"]["mcu"]["mcu_version"], "fw1")
        self.assertEqual(heartbeat["current_print"]["progress"], 0.5)
        self.assertEqual(heartbeat["queue_depth"], 3)


class MessageHandlingTest(unittest.TestCase):
    def test_agent_enqueues_events_from_subscription_update(self):
        with tempfile.TemporaryDirectory() as td:
            config = fleet.FleetConfig(
                printer_id="printer-a",
                backend_url="http://pi:8080",
                auth_token="token",
                queue_db=os.path.join(td, "queue.sqlite3"))
            agent = fleet.FleetMetricsAgent(config)
            agent.handle_klipper_message({
                "params": {
                    "eventtime": 1.0,
                    "status": {
                        "print_stats": {
                            "state": "printing",
                            "filename": "part.gcode",
                        }
                    }
                }
            })
            self.assertEqual(agent.queue.count(), 1)
            batch = agent.queue.pending_batch(1)
            self.assertEqual(batch[0][1]["kind"], "print_started")
            agent.queue.close()


class FakeStore:
    def __init__(self):
        self.heartbeats = []
        self.events = []

    def ingest_heartbeat(self, payload):
        self.heartbeats.append(payload)

    def ingest_events(self, events):
        self.events.extend(events)


class FailingUploader:
    def upload_heartbeat(self, heartbeat):
        raise OSError("backend offline")

    def upload_events(self, printer_id, events):
        raise OSError("backend offline")


class FlushFailureTest(unittest.TestCase):
    def test_upload_failures_do_not_raise_or_drop_events(self):
        with tempfile.TemporaryDirectory() as td:
            config = fleet.FleetConfig(
                printer_id="printer-a",
                backend_url="http://pi:8080",
                auth_token="token",
                queue_db=os.path.join(td, "queue.sqlite3"))
            queue = fleet.EventQueue(config.queue_db)
            queue.enqueue({
                "event_id": "00000000-0000-0000-0000-000000000002",
                "kind": "print_finished",
                "printer_id": "printer-a",
            })
            agent = fleet.FleetMetricsAgent(
                config, queue=queue, uploader=FailingUploader())
            agent.status["webhooks"] = {"state": "ready"}
            agent.flush_once()
            self.assertEqual(queue.count(), 1)
            queue.close()


class BackendTest(unittest.TestCase):
    def setUp(self):
        self.store = FakeStore()
        self.httpd = backend.FleetHTTPServer(
            ("127.0.0.1", 0), self.store, ["secret"])
        self.thread = threading.Thread(target=self.httpd.serve_forever)
        self.thread.daemon = True
        self.thread.start()
        host, port = self.httpd.server_address
        self.base_url = "http://%s:%d" % (host, port)

    def tearDown(self):
        self.httpd.shutdown()
        self.httpd.server_close()
        self.thread.join()

    def post(self, path, payload, token="secret"):
        body = json.dumps(payload).encode("utf-8")
        req = urlrequest.Request(
            self.base_url + path,
            data=body,
            headers={
                "Authorization": "Bearer " + token,
                "Content-Type": "application/json",
            },
            method="POST")
        with urlrequest.urlopen(req, timeout=5.0) as res:
            return res.getcode(), json.loads(res.read().decode("utf-8"))

    def test_backend_accepts_authorized_heartbeat(self):
        status, body = self.post("/api/v1/heartbeat", {
            "printer_id": "printer-a",
            "observed_at": "2026-01-01T00:00:00Z",
        })
        self.assertEqual(status, 200)
        self.assertTrue(body["ok"])
        self.assertEqual(self.store.heartbeats[0]["printer_id"], "printer-a")

    def test_backend_rejects_bad_token(self):
        with self.assertRaises(urlerror.HTTPError) as cm:
            self.post("/api/v1/heartbeat", {"printer_id": "p"}, token="bad")
        self.assertEqual(cm.exception.code, 401)
        self.assertEqual(self.store.heartbeats, [])

    def test_backend_accepts_event_batch(self):
        status, body = self.post("/api/v1/events/batch", {
            "printer_id": "printer-a",
            "events": [{"event_id": "e1", "kind": "printer_state"}],
        })
        self.assertEqual(status, 200)
        self.assertEqual(body["accepted"], 1)
        self.assertEqual(self.store.events[0]["event_id"], "e1")


if __name__ == "__main__":
    unittest.main()
