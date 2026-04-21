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
        
    def on_connect(self, client, userdata, flags, rc):
        super().on_connect(client, userdata, flags, rc)
        self.client.subscribe("robobot/vision/aruco")
        
    def on_message(self, client, userdata, msg):
        topic = msg.topic
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
                
            def do_GET(self):
                if self.path.startswith('/api/aruco/set'):
                    enabled = "1" if "enabled=1" in self.path else "0"
                    node_instance.publish("robobot/vision/enable", enabled)
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(json.dumps({'enabled': bool(int(enabled))}).encode('utf-8'))
                    
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
                    
                else:
                    super().do_GET()

        address = ('', 7123)
        self.httpd = socketserver.TCPServer(address, SimpleRESTHandler)
        self.httpd.allow_reuse_address = True
        
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

