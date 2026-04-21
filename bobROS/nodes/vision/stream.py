import time
import json
import sys
import os
import threading
from threading import Condition
import socketserver
from http import server
import numpy as np
import cv2

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from nodes.mqtt_base import MQTTNode
from lib.vision.shared_mem import SharedImageReader

try:
    import simplejpeg
except ImportError:
    simplejpeg = None

class ReusableTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True

class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/stream/main' or self.path == '/stream/cameratest':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            try:
                while True:
                    with self.server.stream_node.cond:
                        self.server.stream_node.cond.wait()
                        if self.path == '/stream/main':
                            frame = self.server.stream_node.processed_jpeg
                        else:
                            frame = self.server.stream_node.raw_jpeg
                    
                    if frame:
                        self.wfile.write(b'--FRAME\r\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', len(frame))
                        self.end_headers()
                        self.wfile.write(frame)
                        self.wfile.write(b'\r\n')
            except Exception:
                pass
        else:
            self.send_error(404)
            self.end_headers()

class StreamNode(MQTTNode):
    def __init__(self):
        super().__init__("StreamNode")
        self.raw_reader = SharedImageReader("raw_frame", (616, 820, 3))
        self.aruco_reader = SharedImageReader("overlay_aruco", (616, 820, 4))
        self.red_reader = SharedImageReader("overlay_redmask", (616, 820, 4))
        self.bw_reader = SharedImageReader("overlay_bw", (616, 820, 4))

        self.raw_jpeg = None
        self.processed_jpeg = None
        self.cond = Condition()
        
        # Layer states (defaulting ArUco to ON)
        self.layers = {
            "aruco": True,
            "red_mask": False,
            "bw_threshold": False
        }

        self.httpd = ReusableTCPServer(("", 7124), StreamingHandler)
        self.httpd.stream_node = self
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)

    def on_message(self, client, userdata, msg):
        if msg.topic == "robobot/vision/layers/set":
            try:
                data = json.loads(msg.payload.decode())
                self.layers.update(data)
                print(f"[StreamNode] Updated layers: {self.layers}")
            except Exception as e:
                print(f"[StreamNode] Error parsing layers: {e}")

    def on_connect(self, client, userdata, flags, rc):
        super().on_connect(client, userdata, flags, rc)
        self.client.subscribe("robobot/vision/layers/set")

    def run(self):
        print(f"[{self.node_name}] Starting MJPEG server on port 7124...")
        self.http_thread.start()
        
        while self.running:
            raw_img = self.raw_reader.read()
            if raw_img is None:
                time.sleep(0.1)
                continue

            # 1. Start with the raw image as canvas
            canvas = raw_img.copy()

            # 2. Base Overrides (BW Threshold)
            if self.layers.get("bw_threshold"):
                bw_overlay = self.bw_reader.read()
                if bw_overlay is not None:
                    # BW is opaque, so we just take the RGB channels
                    canvas = bw_overlay[:, :, :3]

            # 3. Alpha Blending (Red Mask)
            if self.layers.get("red_mask"):
                red_overlay = self.red_reader.read()
                if red_overlay is not None:
                    self._overlay_blend(canvas, red_overlay)

            # 4. Alpha Blending (ArUco)
            if self.layers.get("aruco"):
                aruco_overlay = self.aruco_reader.read()
                if aruco_overlay is not None:
                    self._overlay_blend(canvas, aruco_overlay)

            # 5. Encode and Update
            try:
                if simplejpeg:
                    raw_jpg = simplejpeg.encode_jpeg(raw_img, quality=70, colorspace='RGB')
                    proc_jpg = simplejpeg.encode_jpeg(canvas, quality=70, colorspace='RGB')
                else:
                    # Fallback to OpenCV encoding
                    _, raw_jpg = cv2.imencode('.jpg', cv2.cvtColor(raw_img, cv2.COLOR_RGB2BGR), [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                    _, proc_jpg = cv2.imencode('.jpg', cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR), [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                    raw_jpg = raw_jpg.tobytes()
                    proc_jpg = proc_jpg.tobytes()

                with self.cond:
                    self.raw_jpeg = raw_jpg
                    self.processed_jpeg = proc_jpg
                    self.cond.notify_all()
            except Exception as e:
                pass

            time.sleep(0.02) # Cap at ~50fps logic

        self.httpd.shutdown()

    def _overlay_blend(self, canvas, overlay):
        """Fast manual alpha blend where alpha=255 pixels overwrite the canvas."""
        alpha = overlay[:, :, 3]
        mask = alpha > 0
        canvas[mask] = overlay[mask, :3]

def start_stream():
    node = StreamNode()
    node.start()
