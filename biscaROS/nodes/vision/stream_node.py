# biscaROS/nodes/vision/stream_node.py
import time
import threading
import numpy as np
import cv2 as cv
from multiprocessing import shared_memory
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
from biscaROS.core.base_node import BaseNode

# --- MJPEG HTTP Server Setup ---
class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in separate threads, automatically cleaning up dead ones."""
    allow_reuse_address = True
    daemon_threads = True

class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.startswith('/stream'):
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            try:
                while True:
                    # Wait for the render thread to broadcast a new frame
                    with self.server.stream_node.frame_condition:
                        self.server.stream_node.frame_condition.wait()
                        jpeg_bytes = self.server.stream_node.latest_jpeg
                        
                    if jpeg_bytes:
                        # Safely write the raw MJPEG multipart headers
                        header = f"--FRAME\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg_bytes)}\r\n\r\n"
                        self.wfile.write(header.encode('utf-8'))
                        self.wfile.write(jpeg_bytes)
                        self.wfile.write(b'\r\n')
            except Exception:
                # Client disconnected (e.g. closed browser tab)
                pass
        else:
            self.send_error(404)
            self.end_headers()

# --- Main Stream Node ---
class StreamNode(BaseNode):
    def __init__(self):
        super().__init__(node_name="stream_compositor_node")
        
        self.frame_shape = (616, 820, 3)
        self.frame_size = int(np.prod(self.frame_shape))
        
        # 1. Attach to Camera SHM (Input Base)
        print("[StreamNode] Waiting for camera shared memory...", flush=True)
        while True:
            try:
                self.cam_shm = shared_memory.SharedMemory(name="biscaROS_camera_frame")
                self.cam_array = np.ndarray(self.frame_shape, dtype=np.uint8, buffer=self.cam_shm.buf)
                print("[StreamNode] Connected to camera memory.", flush=True)
                break
            except FileNotFoundError:
                time.sleep(0.5) # Wait for camera_node to allocate it
            
        # 2. Overlay Tracker
        self.overlays = {
            "biscaROS_aruco_overlay": {"shm": None, "array": None, "last_seen": 0},
            "biscaROS_ball_overlay": {"shm": None, "array": None, "last_seen": 0}
        }
        self.overlay_timeout = 1.0 

        self.composite_buffer = np.zeros(self.frame_shape, dtype=np.uint8)
        
        # 3. Broadcasting variables (The Fix)
        self.latest_jpeg = None
        self.frame_condition = threading.Condition()
        
        self.client.subscribe("biscaROS/vision/overlays/ready")

        # 4. Start the central rendering loop
        self.render_thread = threading.Thread(target=self._render_loop, daemon=True)
        self.render_thread.start()

    def _on_message(self, client, userdata, msg):
        if msg.topic == "biscaROS/vision/overlays/ready":
            overlay_name = msg.payload.decode('utf-8')
            if overlay_name in self.overlays:
                self.overlays[overlay_name]["last_seen"] = time.time()
                if self.overlays[overlay_name]["shm"] is None:
                    try:
                        shm = shared_memory.SharedMemory(name=overlay_name)
                        arr = np.ndarray(self.frame_shape, dtype=np.uint8, buffer=shm.buf)
                        self.overlays[overlay_name]["shm"] = shm
                        self.overlays[overlay_name]["array"] = arr
                        print(f"[StreamNode] Successfully attached to {overlay_name}", flush=True)
                    except FileNotFoundError:
                        pass 

    def _render_loop(self):
        """Runs continuously in the background, compositing exactly 1 frame for all clients."""
        encode_param = [int(cv.IMWRITE_JPEG_QUALITY), 80]
        
        while True:
            # Base frame
            np.copyto(self.composite_buffer, self.cam_array)
            
            current_time = time.time()
            # Add active overlays
            for name, data in self.overlays.items():
                if data["array"] is not None and (current_time - data["last_seen"]) < self.overlay_timeout:
                    cv.add(self.composite_buffer, data["array"], dst=self.composite_buffer)
            
            # Convert to BGR and encode
            bgr_composite = cv.cvtColor(self.composite_buffer, cv.COLOR_RGB2BGR)
            success, jpeg = cv.imencode('.jpg', bgr_composite, encode_param)
            
            if success:
                # Lock the condition, update the frame, and broadcast it to ALL web clients
                with self.frame_condition:
                    self.latest_jpeg = jpeg.tobytes()
                    self.frame_condition.notify_all()
            
            # Target ~30 FPS
            time.sleep(0.033)

    def start_server(self, port=7124):
        # Binding to '' means it answers to both localhost and the 10.197... IP
        server = ThreadedHTTPServer(('', port), MJPEGHandler)
        server.stream_node = self
        
        print(f"[StreamNode] Serving MJPEG stream on port {port}...", flush=True)
        
        server_thread = threading.Thread(target=server.serve_forever, daemon=True)
        server_thread.start()
        
        return server

    def stop(self):
        print("[StreamNode] Shutting down...")
        for name, data in self.overlays.items():
            if data["shm"] is not None:
                data["shm"].close()
        super().stop()

if __name__ == "__main__":
    node = StreamNode()
    node.start() 
    http_server = node.start_server(port=7124)
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        http_server.shutdown()
        node.stop()