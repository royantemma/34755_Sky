"""
The stream node combines visual overlays from the detection algorithms, makes a composite image, and adds it to a MJPEG stream that can be viewed on a web browser.
"""

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
    """Handle requests in a separate thread to allow multiple web viewers."""
    pass

class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/stream':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    # Fetch the latest composited JPEG from the main node
                    jpeg_bytes = self.server.stream_node.get_composite_jpeg()
                    if jpeg_bytes:
                        self.wfile.write(b'--FRAME\r\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', len(jpeg_bytes))
                        self.end_headers()
                        self.wfile.write(jpeg_bytes)
                        self.wfile.write(b'\r\n')
                    time.sleep(0.033) # Limit to ~30 FPS to save bandwidth
            except Exception as e:
                # Client disconnected or network error
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
        try:
            self.cam_shm = shared_memory.SharedMemory(name="biscaROS_camera_frame")
            self.cam_array = np.ndarray(self.frame_shape, dtype=np.uint8, buffer=self.cam_shm.buf)
        except FileNotFoundError:
            print("[StreamNode] Error: Camera shared memory not found. Start camera node first.")
            exit(1)
            
        # 2. Overlay Tracker (Dynamic SHM attachment)
        # Keeps track of overlays, their SHM arrays, and when they were last updated
        self.overlays = {
            "biscaROS_aruco_overlay": {"shm": None, "array": None, "last_seen": 0},
            "biscaROS_ball_overlay": {"shm": None, "array": None, "last_seen": 0}
            # You can add bw_threshold or others here seamlessly
        }
        self.overlay_timeout = 1.0 # Seconds before dropping a stale overlay

        # 3. Pre-allocate buffer for the final frame to avoid memory thrashing
        self.composite_buffer = np.zeros(self.frame_shape, dtype=np.uint8)
        
        # Subscribe to overlay notifications
        self.client.subscribe("biscaROS/vision/overlays/ready")

    def _on_message(self, client, userdata, msg):
        if msg.topic == "biscaROS/vision/overlays/ready":
            overlay_name = msg.payload.decode('utf-8')
            
            if overlay_name in self.overlays:
                self.overlays[overlay_name]["last_seen"] = time.time()
                
                # Dynamically attach to the overlay SHM if we haven't yet
                if self.overlays[overlay_name]["shm"] is None:
                    try:
                        shm = shared_memory.SharedMemory(name=overlay_name)
                        arr = np.ndarray(self.frame_shape, dtype=np.uint8, buffer=shm.buf)
                        self.overlays[overlay_name]["shm"] = shm
                        self.overlays[overlay_name]["array"] = arr
                        print(f"[StreamNode] Successfully attached to {overlay_name}")
                    except FileNotFoundError:
                        pass # Node notified us, but memory isn't ready yet. Will try again next frame.

    def get_composite_jpeg(self):
        """Builds the current frame by adding active overlays, then encodes to JPEG."""
        # 1. Start with the raw camera frame
        np.copyto(self.composite_buffer, self.cam_array)
        
        current_time = time.time()
        
        # 2. Add active overlays
        for name, data in self.overlays.items():
            if data["array"] is not None:
                # Check if the vision node is still alive and updating
                if (current_time - data["last_seen"]) < self.overlay_timeout:
                    # OpenCV additive blending. Black pixels (0,0,0) do nothing.
                    # Colored pixels (e.g., Aruco axes) are added directly on top.
                    cv.add(self.composite_buffer, data["array"], dst=self.composite_buffer)
        
        # 3. Convert to RGB for web viewing (OpenCV uses BGR natively, but our SHMs are stored as RGB)
        # Assuming camera_node and vision nodes output RGB arrays to SHM, 
        # OpenCV's imencode actually expects BGR. So we flip it before encoding.
        bgr_composite = cv.cvtColor(self.composite_buffer, cv.COLOR_RGB2BGR)
        
        # 4. Compress to JPEG
        # Optimization: Lower quality slightly for higher framerates over WiFi (default is 95)
        encode_param = [int(cv.IMWRITE_JPEG_QUALITY), 80]
        success, jpeg = cv.imencode('.jpg', bgr_composite, encode_param)
        
        if success:
            return jpeg.tobytes()
        return None

    def start_server(self, port=7124):
        # Link this node instance to the HTTP server so the handler can call get_composite_jpeg()
        server = ThreadedHTTPServer(('0.0.0.0', port), MJPEGHandler)
        server.stream_node = self
        
        print(f"[StreamNode] Serving MJPEG stream on port {port} at /stream")
        
        # Run server in a background thread
        server_thread = threading.Thread(target=server.serve_forever)
        server_thread.daemon = True
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
    node.start() # Start MQTT loop
    http_server = node.start_server(port=7124)
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        http_server.shutdown()
        node.stop()