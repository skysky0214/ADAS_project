import json
import math
import os
import sys
import queue
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

# Thread-safe global list of client queues
subscribers = []
subscribers_lock = threading.Lock()


def find_project_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "app" / "main.py").exists() and (parent / "tools").exists():
            return parent
    return Path(__file__).resolve().parents[3]


PROJECT_ROOT = find_project_root()
ROI_CONFIG_PATH = PROJECT_ROOT / "artifacts" / "runtime_roi_config.json"
ROI_CONFIG_DEFAULTS = {
    "static_roi_x_min": 2.50,
    "static_roi_x_max": 15.00,
    "static_roi_y_min": -1.10,
    "static_roi_y_max": 1.10,
    "static_roi_z_min": -0.90,
    "static_roi_z_max": 0.00,
}
roi_config_lock = threading.Lock()

class DashboardHTTPRequestHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        # Set the directory to serve files from (the parent of this file or explicitly web_dashboard/)
        current_dir = Path(__file__).resolve().parent
        super().__init__(*args, directory=str(current_dir), **kwargs)

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(200, "ok")
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header("Access-Control-Allow-Headers", "X-Requested-With, Content-Type")
        self.end_headers()

    def do_GET(self):
        path = urlparse(self.path).path
        # Match Server-Sent Events subscription endpoint
        if path == '/stream':
            self.handle_sse_stream()
            return
        if path == '/api/roi':
            self.handle_get_roi()
            return

        # Standard static file serving
        super().do_GET()

    def do_POST(self):
        path = urlparse(self.path).path
        # Match frame broadcast endpoint from ADAS pipeline node
        if path == '/api/frame':
            self.handle_post_frame()
            return
        if path == '/api/roi':
            self.handle_post_roi()
            return

        self.send_error(404, "Endpoint not found")

    def handle_sse_stream(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

        # Keep only the newest frame per client so stale warnings cannot backlog.
        q = queue.Queue(maxsize=1)

        with subscribers_lock:
            subscribers.append(q)
            print(f"[DashboardServer] Client connected to live stream. Active clients: {len(subscribers)}")

        try:
            # Continuously wait for frame packets and stream them as SSE chunks
            while True:
                try:
                    # Block on queue for next frame
                    frame_data = q.get(timeout=1.0)
                    sse_line = f"data: {frame_data}\n\n"
                    self.wfile.write(sse_line.encode('utf-8'))
                    self.wfile.flush()
                except queue.Empty:
                    # Write simple SSE heartbeat keepalive ping to prevent timeouts
                    self.wfile.write(b": ping\n\n")
                    self.wfile.flush()
        except (ConnectionError, BrokenPipeError):
            print("[DashboardServer] Client connection lost.")
        finally:
            with subscribers_lock:
                if q in subscribers:
                    subscribers.remove(q)
                    print(f"[DashboardServer] Client disconnected. Active clients: {len(subscribers)}")

    def handle_post_frame(self):
        content_length = int(self.headers.get('Content-Length', 0))
        post_data = self.rfile.read(content_length)

        try:
            # Validate JSON formatting
            frame_json = post_data.decode('utf-8')
            json.loads(frame_json) # simple parse verification

            # Broadcast frame to all active SSE listener queues
            with subscribers_lock:
                inactive = []
                for q in subscribers:
                    try:
                        while True:
                            try:
                                q.get_nowait()
                            except queue.Empty:
                                break
                        q.put_nowait(frame_json)
                    except queue.Full:
                        inactive.append(q)

                # Cleanup blocked/full queues
                for q in inactive:
                    if q in subscribers:
                        subscribers.remove(q)

            # Respond to POST source (pipeline node)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "success", "subscribers": len(subscribers)}).encode('utf-8'))

        except Exception as e:
            self.send_response(400)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"error": str(e)}).encode('utf-8'))

    def handle_get_roi(self):
        self.send_json(200, {"config": load_roi_config(), "path": str(ROI_CONFIG_PATH)})

    def handle_post_roi(self):
        content_length = int(self.headers.get('Content-Length', 0))
        post_data = self.rfile.read(content_length)
        try:
            payload = json.loads(post_data.decode('utf-8'))
            config = coerce_roi_config(payload)
            save_roi_config(config)
            self.send_json(200, {"status": "success", "config": config, "path": str(ROI_CONFIG_PATH)})
        except Exception as e:
            self.send_json(400, {"error": str(e)})

    def send_json(self, status, payload):
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(payload, ensure_ascii=False).encode('utf-8'))


def load_roi_config():
    with roi_config_lock:
        config = dict(ROI_CONFIG_DEFAULTS)
        try:
            saved = json.loads(ROI_CONFIG_PATH.read_text(encoding="utf-8"))
            if isinstance(saved, dict):
                config.update(coerce_roi_config(saved))
        except FileNotFoundError:
            pass
        return config


def coerce_roi_config(payload):
    if not isinstance(payload, dict):
        raise ValueError("ROI payload must be a JSON object")
    config = {}
    for key, default in ROI_CONFIG_DEFAULTS.items():
        value = payload.get(key, default)
        try:
            numeric_value = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Invalid ROI value for {key}: {value!r}") from exc
        if not math.isfinite(numeric_value):
            raise ValueError(f"Invalid ROI value for {key}: {value!r}")
        config[key] = numeric_value
    for min_key, max_key in (
        ("static_roi_x_min", "static_roi_x_max"),
        ("static_roi_y_min", "static_roi_y_max"),
        ("static_roi_z_min", "static_roi_z_max"),
    ):
        if config[min_key] > config[max_key]:
            raise ValueError(f"Invalid ROI bounds: {min_key} must be <= {max_key}")
    return config


def save_roi_config(config):
    with roi_config_lock:
        ROI_CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = ROI_CONFIG_PATH.with_suffix(".json.tmp")
        tmp_path.write_text(json.dumps(config, ensure_ascii=False, indent=2), encoding="utf-8")
        tmp_path.replace(ROI_CONFIG_PATH)

def run_server(port=8000):
    # Use standard Python ThreadingHTTPServer for simple, zero-dependency concurrent requests
    server_address = ('', port)
    httpd = ThreadingHTTPServer(server_address, DashboardHTTPRequestHandler)
    print("\n==========================================================")
    print("  ADAS INFOTAINMENT NAVIGATION SYSTEM RUNNING AT:")
    print(f"  URL: http://localhost:{port}")
    print("==========================================================\n")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[DashboardServer] Shutting down...")
        httpd.server_close()
        sys.exit(0)

if __name__ == '__main__':
    port = 8000
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            pass
    run_server(port)
