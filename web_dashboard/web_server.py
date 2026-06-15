import json
import os
import sys
import queue
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

# Thread-safe global list of client queues
subscribers = []
subscribers_lock = threading.Lock()

class DashboardHTTPRequestHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        # Set the directory to serve files from (the parent of this file or explicitly web_dashboard/)
        current_dir = Path(__file__).resolve().parent
        super().__init__(*args, directory=str(current_dir), **kwargs)

    def do_OPTIONS(self):
        self.send_response(200, "ok")
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header("Access-Control-Allow-Headers", "X-Requested-With, Content-Type")
        self.end_headers()

    def do_GET(self):
        # Match Server-Sent Events subscription endpoint
        if self.path == '/stream':
            self.handle_sse_stream()
            return
        
        # Standard static file serving
        super().do_GET()

    def do_POST(self):
        # Match frame broadcast endpoint from ADAS pipeline node
        if self.path == '/api/frame':
            self.handle_post_frame()
            return
            
        self.send_error(404, "Endpoint not found")

    def handle_sse_stream(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

        # Create a queue specifically for this connection
        q = queue.Queue(maxsize=100)
        
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
