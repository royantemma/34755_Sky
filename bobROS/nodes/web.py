import time
import os
import json
from http import server
import socketserver
from nodes.mqtt_base import MQTTNode
import threading

class WebNode(MQTTNode):
    def __init__(self):
        super().__init__("WebNode")
        self.latest_aruco_data = {}
        self.latest_jpeg = None
        self.cond = threading.Condition()
        
    def on_connect(self, client, userdata, flags, rc):
        super().on_connect(client, userdata, flags, rc)
        self.client.subscribe("robobot/vision/aruco")
        self.client.subscribe("robobot/vision/image")
        
    def on_message(self, client, userdata, msg):
        topic = msg.topic
        
        if topic == "robobot/vision/image":
            with self.cond:
                self.latest_jpeg = msg.payload
                self.cond.notify_all()
            return
            
        payload = msg.payload.decode('utf-8')
        
        if topic == "robobot/vision/aruco":
            self.latest_aruco_data = json.loads(payload)

    def run(self):
        print(f"[{self.node_name}] Starting the Web UI API on Port 7123...")
        
        node_instance = self
        web_dir = os.path.join(os.path.dirname(__file__), "..", "web_assets")

        class SimpleRESTHandler(server.SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=web_dir, **kwargs)
                
            def log_message(self, format, *args):
                # Suppress spammy log outputs from GET requests
                pass
                
            def do_GET(self):
                if self.path.startswith('/api/aruco/set'):
                    enabled = "1" if "enabled=1" in self.path else "0"
                    node_instance.publish("robobot/vision/enable", enabled)
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(json.dumps({'enabled': bool(int(enabled))}).encode('utf-8'))
                
                elif self.path == '/api/vision/layers':
                    # Only used for POST, but just in case
                    self.send_error(405)

                elif self.path == '/api/aruco/data':
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(json.dumps(node_instance.latest_aruco_data).encode('utf-8'))
                    
                elif self.path == '/api/aruco/catch':
                    # Launch mission via MQTT rather than subprocess!
                    cmd = json.dumps({"command": "start", "name": "cube_catch"})
                    node_instance.publish("robobot/cmd/mission", cmd)
                    
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(json.dumps({"status": "catching"}).encode('utf-8'))
                    
                elif self.path == '/api/aruco/stop':
                    # Stop mission via MQTT
                    cmd = json.dumps({"command": "stop"})
                    node_instance.publish("robobot/cmd/mission", cmd)
                    
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(json.dumps({"status": "stopped"}).encode('utf-8'))
                    
                elif self.path in ('/stream/main', '/stream/cameratest'):
                    self.send_response(200)
                    self.send_header('Age', 0)
                    self.send_header('Cache-Control', 'no-cache, private')
                    self.send_header('Pragma', 'no-cache')
                    self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
                    self.end_headers()
                    try:
                        while node_instance.running:
                            with node_instance.cond:
                                node_instance.cond.wait(timeout=1.5)
                                jpeg = node_instance.latest_jpeg
                                
                            if jpeg:
                                self.wfile.write(b'--FRAME\r\n')
                                self.send_header('Content-Type', 'image/jpeg')
                                self.send_header('Content-Length', len(jpeg))
                                self.end_headers()
                                self.wfile.write(jpeg)
                                self.wfile.write(b'\r\n')
                    except Exception:
                        pass

            def do_POST(self):
                if self.path == '/api/vision/layers':
                    content_length = int(self.headers['Content-Length'])
                    post_data = self.rfile.read(content_length)
                    # Relay to MQTT
                    node_instance.publish("robobot/vision/layers/set", post_data.decode('utf-8'))
                    
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(json.dumps({"status": "ok"}).encode('utf-8'))
                else:
                    self.send_error(404)

        class ReusableTCPServer(socketserver.TCPServer):
            allow_reuse_address = True

        address = ('', 7123)
        self.httpd = ReusableTCPServer(address, SimpleRESTHandler)
        
        # Run HTTP server in a sub-thread
        httpd_thread = threading.Thread(target=self.httpd.serve_forever)
        httpd_thread.daemon = True
        httpd_thread.start()

        while self.running:
            time.sleep(1)
            
        self.httpd.shutdown()

def start_web():
    node = WebNode()
    node.start()

