"""
The web node starts the web server (using the web_assets), and sends MQTT messages when button are pressed, information entered...
"""

# biscaROS/nodes/web_node.py
import os
import json
import time
import threading
from http.server import SimpleHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from biscaROS.core.base_node import BaseNode

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    pass

class WebHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        # Serve static files directly from web_assets
        web_dir = os.path.join(os.path.dirname(__file__), '..', 'web_assets')
        super().__init__(*args, directory=web_dir, **kwargs)

    def do_GET(self):
        if self.path == '/api/data':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            # Serialize the node's current state buffer
            self.wfile.write(json.dumps(self.server.node.web_state).encode())
        else:
            # Fallback to serving static files (index.html, style.css, etc.)
            super().do_GET()

    def do_POST(self):
        content_length = int(self.headers.get('Content-Length', 0))
        post_data = self.rfile.read(content_length) if content_length > 0 else b""
        
        # 1. Mission Control
        if self.path == '/api/cmd/mission':
            data = json.loads(post_data)
            mission_name = data.get("mission")
            self.server.node.publish("biscaROS/mission/cmd", f"start:{mission_name}")
            self.send_response(200); self.end_headers()

        elif self.path == '/api/cmd/abort':
            self.server.node.publish("biscaROS/mission/cmd", "abort")
            self.send_response(200); self.end_headers()

        # 2. Hardware Control
        elif self.path == '/api/cmd/servo':
            self.server.node.publish("robobot/cmd/T0", "servo 1 disable")
            self.send_response(200); self.end_headers()

        # 3. Web UI State Toggles
        elif self.path == '/api/cmd/aruco_toggle':
            data = json.loads(post_data)
            self.server.node.web_state["aruco_enabled"] = data.get("enabled", True)
            self.send_response(200); self.end_headers()
            
        elif self.path == '/api/vision/layers':
            # Publish requested layers to stream compositor or vision nodes if needed
            layers = json.loads(post_data)
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