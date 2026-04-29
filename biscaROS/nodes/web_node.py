"""
The web node starts the web server (using the web_assets), and sends MQTT messages when button are pressed, information entered...
"""

# biscaROS/nodes/web_node.py
import os
import json
import time
import threading
import urllib.request
from http.server import SimpleHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from biscaROS.core.base_node import BaseNode

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

class WebHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        web_dir = os.path.join(os.path.dirname(__file__), '..', 'web_assets')
        super().__init__(*args, directory=web_dir, **kwargs)

    def log_message(self, format, *args):
        pass # get rid of default logging noise

    def do_GET(self):
        if self.path == '/api/data':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(self.server.node.web_state).encode())
            
        # --- NEW PURE TCP PROXY ---
        elif self.path.startswith('/stream'):
            import socket
            proxy_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                # 1. Connect to the internal stream node
                proxy_sock.settimeout(2.0)
                proxy_sock.connect(('127.0.0.1', 7124))
                proxy_sock.settimeout(None) # Remove timeout for continuous video
                
                # 2. Send a raw GET request
                proxy_sock.sendall(b"GET /stream HTTP/1.1\r\nHost: 127.0.0.1\r\nConnection: close\r\n\r\n")
                
                # 3. Pipe every byte directly to the browser (self.request is the client socket)
                while True:
                    chunk = proxy_sock.recv(8192)
                    if not chunk:
                        break
                    self.request.sendall(chunk)
                    
            except Exception:
                pass # The user closed the browser tab
            finally:
                proxy_sock.close()
                self.close_connection = True # Tell Python we hijacked the connection
            return # Exit handler immediately
        # ---------------------------
        
        else:
            # Fallback to serving static files (index.html, style.css, etc.)
            super().do_GET()

    def do_POST(self):
        content_length = int(self.headers.get('Content-Length', 0))
        post_data = self.rfile.read(content_length) if content_length > 0 else b""
        
        if self.path == '/api/cmd/mission':
            data = json.loads(post_data)
            mission_name = data.get("mission")
            print(f"[WebNode] CMD Received: Start Mission -> {mission_name}", flush=True)
            self.server.node.publish("biscaROS/mission/cmd", f"start:{mission_name}")
            self.send_response(200); self.end_headers()

        elif self.path == '/api/cmd/abort':
            print("[WebNode] CMD Received: ABORT", flush=True)
            self.server.node.publish("biscaROS/mission/cmd", "abort")
            self.send_response(200); self.end_headers()

        elif self.path == '/api/cmd/servo':
            print("[WebNode] CMD Received: Disable Servo", flush=True)
            self.server.node.publish("robobot/cmd/T0", "servo 1 disable")
            self.send_response(200); self.end_headers()

        elif self.path == '/api/cmd/aruco_toggle':
            data = json.loads(post_data)
            state = data.get("enabled", True)
            print(f"[WebNode] CMD Received: Set ArUco Detection -> {state}", flush=True)
            self.server.node.web_state["aruco_enabled"] = state
            self.send_response(200); self.end_headers()
            
        elif self.path == '/api/vision/layers':
            layers = json.loads(post_data)
            print(f"[WebNode] CMD Received: Update Vision Layers", flush=True)
            self.server.node.publish("biscaROS/vision/layers/cmd", json.dumps(layers))
            self.send_response(200); self.end_headers()

        else:
            self.send_error(404)

class WebNode(BaseNode):
    def __init__(self):
        super().__init__(node_name="web_server_node")
        
        self.web_state = {
            "aruco_enabled": True,
            "aruco_data": {}
        }
        
        self.client.subscribe("biscaROS/vision/aruco/data")

    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode('utf-8')
        
        if topic == "biscaROS/vision/aruco/data":
            try:
                if self.web_state["aruco_enabled"]:
                    self.web_state["aruco_data"] = json.loads(payload)
                else:
                    self.web_state["aruco_data"] = {}
            except json.JSONDecodeError:
                pass

    def start_server(self, port=7123):
        server = ThreadedHTTPServer(('0.0.0.0', port), WebHandler)
        server.node = self
        
        print(f"[WebNode] Serving UI on port {port}")
        server_thread = threading.Thread(target=server.serve_forever)
        server_thread.daemon = True
        server_thread.start()
        return server

if __name__ == "__main__":
    node = WebNode()
    node.start()
    http_server = node.start_server(port=7123)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        http_server.shutdown()
        node.stop()