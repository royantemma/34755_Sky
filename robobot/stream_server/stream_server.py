#!/usr/bin/env python3
# Rui Santos & Sara Santos - Random Nerd Tutorials
# Complete project details at https://RandomNerdTutorials.com/raspberry-pi-mjpeg-streaming-web-server-picamera2/

# Mostly copied from https://picamera.readthedocs.io/en/release-1.13/recipes2.html
# Run this script, then point a web browser at http:<this-ip-address>:7123
# Note: needs simplejpeg to be installed (pip3 install simplejpeg).

import io
import logging
import socketserver
import socket
import threading
import simplejpeg
import numpy as np
import cv2 as cv
import json
import os
import time

from http import server
from threading import Condition
from setproctitle import setproctitle

from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput

# set title of process, so that it is not just called Python
setproctitle("stream_server")

hostname = socket.gethostname()

# --- ArUco Processing Globals ---
latest_frame = None
latest_frame_cond = Condition()
aruco_enabled = False
aruco_data = []

class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


class StreamManager:
    def __init__(self):
        self.streams = {}  # name: {output, enabled}

    def add_stream(self, name):
        """Register a new stream and return its output object."""
        self.streams[name] = {'output': StreamingOutput(), 'enabled': True}
        return self.streams[name]['output']

    def set_enabled(self, name, enabled=True):
        """Enable or disable a stream dynamically."""
        if name in self.streams:
            self.streams[name]['enabled'] = enabled

    def get_output(self, name):
        """Return the output object if enabled, else None."""
        if name in self.streams and self.streams[name]['enabled']:
            return self.streams[name]['output']
        return None

# Global instance to be imported anywhere
stream_manager = StreamManager()


class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            try:
                curr_dir = os.path.dirname(os.path.abspath(__file__))
                with open(os.path.join(curr_dir, 'index.html'), 'rb') as f:
                    content = f.read()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html')
                self.send_header('Content-Length', len(content))
                self.end_headers()
                self.wfile.write(content)
            except Exception as e:
                self.send_error(500, f"Error loading index.html: {e}")
        elif self.path.startswith('/api/aruco/set'):
            global aruco_enabled
            if 'enabled=1' in self.path:
                aruco_enabled = True
            elif 'enabled=0' in self.path:
                aruco_enabled = False
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({'enabled': aruco_enabled}).encode('utf-8'))
        elif self.path == '/api/aruco/data':
            response = {
                'enabled': aruco_enabled,
                'markers': aruco_data
            }
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(response).encode('utf-8'))
        elif self.path.startswith('/stream/'):
            # Get stream name
            stream_name = self.path.split('/')[-1]
            output = stream_manager.get_output(stream_name)
            if not output:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

main_output = stream_manager.add_stream("main")
cameratest_output = stream_manager.add_stream("cameratest")

def process_frames():
    global latest_frame
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(main={"size": (820, 616)},controls={'FrameDurationLimits': (200000, 500000)}))
    picam2.start()
    
    while True:
        frame = picam2.capture_array()

        # Picamera2 XBGR8888 is RGBX in memory order — drop the padding channel.
        if frame.shape[2] == 4:
            frame = np.ascontiguousarray(frame[:, :, :3])
            
        with latest_frame_cond:
            latest_frame = frame
            latest_frame_cond.notify_all()

        jpeg = simplejpeg.encode_jpeg(frame, quality=80, colorspace='RGB')
        main_output.write(jpeg)

def aruco_worker():
    global latest_frame, aruco_enabled, aruco_data
    
    # Try to load intrinsic calibration data
    calib_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "mqtt_python")
    calib_file = os.path.join(calib_dir, "calibration_checkerboard.npz")
    fallback_file = os.path.join(calib_dir, "calibration_data.npz")
    
    mtx = None
    dist = None
    
    if os.path.exists(calib_file):
        with np.load(calib_file) as X:
            mtx, dist = [X[i] for i in ('mtx', 'dist')]
    elif os.path.exists(fallback_file):
        with np.load(fallback_file) as X:
            mtx, dist = [X[i] for i in ('mtx', 'dist')]
            
    marker_size = 0.035
    obj_points = np.array([
        [-marker_size/2,  marker_size/2, 0],
        [ marker_size/2,  marker_size/2, 0],
        [ marker_size/2, -marker_size/2, 0],
        [-marker_size/2, -marker_size/2, 0]
    ], dtype=np.float32)

    aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
    if hasattr(cv.aruco, 'DetectorParameters_create'):
        aruco_params = cv.aruco.DetectorParameters_create()
    else:
        aruco_params = cv.aruco.DetectorParameters()

    while True:
        if not aruco_enabled:
            time.sleep(0.1)
            continue
            
        with latest_frame_cond:
            latest_frame_cond.wait(timeout=0.2)
            if latest_frame is None:
                continue
            frame = latest_frame.copy()
            
        current_data = []
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        
        try:
            detector = cv.aruco.ArucoDetector(aruco_dict, aruco_params)
            corners, ids, _ = detector.detectMarkers(gray)
        except AttributeError:
            corners, ids, _ = cv.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
            
        if ids is not None and mtx is not None:
            cv.aruco.drawDetectedMarkers(frame, corners, ids)
            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                marker_corners = corners[i][0]
                ret, rvec, tvec = cv.solvePnP(obj_points, marker_corners, mtx, dist)
                if ret:
                    cv.drawFrameAxes(frame, mtx, dist, rvec, tvec, marker_size)
                    x, y, z = tvec.flatten()
                    x_mm, y_mm, z_mm = x * 1000, y * 1000, z * 1000
                    
                    # Apply Extrinsic calibration (Pitch = 5.36 deg, Z = 189.2 mm)
                    theta = np.radians(5.36)
                    cam_z_offset = 189.2
                    
                    x_rob = x_mm
                    y_rob = z_mm * np.cos(theta) - y_mm * np.sin(theta)
                    z_rob = cam_z_offset - (y_mm * np.cos(theta) + z_mm * np.sin(theta))
                    
                    current_data.append({
                        "id": marker_id,
                        "x": round(x_rob, 1),
                        "y": round(y_rob, 1),
                        "z": round(z_rob, 1)
                    })
                    
        aruco_data = current_data
        
        # Push annotated frame to cameratest_output
        jpeg_bw = simplejpeg.encode_jpeg(frame, quality=80, colorspace='RGB')
        cameratest_output.write(jpeg_bw)

def start_stream_server():
    threading.Thread(target=process_frames, daemon=True).start()
    threading.Thread(target=aruco_worker, daemon=True).start()

    address = ('', 7123)
    server = StreamingServer(address, StreamingHandler)
    server.serve_forever()


if __name__ == "__main__":
    try:
        start_stream_server()
    finally:
        pass
