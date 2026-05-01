#!/usr/bin/env python3
# Rui Santos & Sara Santos - Random Nerd Tutorials
# Complete project details at https://RandomNerdTutorials.com/raspberry-pi-mjpeg-streaming-web-server-picamera2/

# Mostly copied from https://picamera.readthedocs.io/en/release-1.13/recipes2.html
# Run this script, then point a web browser at http:<this-ip-address>:7123
# Note: needs simplejpeg to be installed (pip3 install simplejpeg).

# ps aux | grep stream_server
# kill xxx
# python robobot/stream_server/stream_server.py

import io
import logging
import socketserver
import socket
import threading
from unittest import runner
import simplejpeg
import numpy as np
import cv2 as cv
import json
import os
import sys
import time


# Add mqtt_python to path so we can import aruco module
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "mqtt_python"))
import aruco

from mission_runner import MissionRunner
from missions.mission_all_v1_0 import TASKS, TOTAL_TIME, GOAL_TIME_BUFFER
from sfuse import iwo
# import missions.current_mission as current_mission
# from mqtt_python.mission_runner import MissionRunner
# from ..mqtt_python.mission_runner import MissionRunner

# import mqtt_python.missions.current_mission as current_mission
# from ..mqtt_python.missions.current_mission import TASKS, TOTAL_TIME, GOAL_TIME_BUFFER


from http import server
from threading import Condition
from setproctitle import setproctitle

from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput


# Mission runner thread and runner
mission_thread = None
current_runner = None

TOTAL_TIME = 0
GOAL_TIME_BUFFER = 0
TASKS = []

iwo_data = []


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

# ADD TO THE IF STATEMENT TO EXTEND FUNCTIONALITY
class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):

        global mission_thread, current_runner

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
            # Also send estimates
            est = aruco_data.get('estimates', []) if isinstance(aruco_data, dict) else []
            
            response = {
                'enabled': aruco_enabled,
                'markers': aruco_data.get('markers', []) if isinstance(aruco_data, dict) else aruco_data,
                'cube_center': aruco_data.get('cube_center', None) if isinstance(aruco_data, dict) else None,
                'estimates': est
            }

            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(response).encode('utf-8'))
        elif self.path == '/api/aruco/catch':
            import subprocess
            # Launch the mission script correctly as a separate process from the mqtt_python directory
            mqtt_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "mqtt_python")
            def run_mission():
                subprocess.run(["python3", "mqtt-client.py", "-cc"], cwd=mqtt_dir)
            threading.Thread(target=run_mission, daemon=True).start()
            
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "catching"}).encode('utf-8'))
        elif self.path == '/api/aruco/stop':
            # Stop any running mqtt-client
            os.system("pkill -f mqtt-client")
            # Clear any frozen velocity control
            os.system("mosquitto_pub -t 'robobot/cmd/ti' -m 'rc 0 0'")
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "stopped"}).encode('utf-8'))
        elif self.path == '/api/servo/disable':
            # Clear any frozen velocity control
            os.system("mosquitto_pub -t 'robobot/cmd/T0' -m 'servo 1 10000 0'")
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "servo disabled"}).encode('utf-8'))
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

        elif self.path == '/api/mission/start':

            # Don't start if already running
            if mission_thread and mission_thread.is_alive():
                self.send_response(409)  # 409 = Conflict
                self.end_headers()
                self.wfile.write(json.dumps({'error': 'Mission already running'}).encode('utf-8'))
                return

            def run():
                runner = MissionRunner(TASKS, TOTAL_TIME, GOAL_TIME_BUFFER)
                current_runner = runner
                runner.run()
                current_runner = None
            
            mission_thread = threading.Thread(target=run, daemon=True)
            mission_thread.start()

            # reply to the browser
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({'Start'}).encode('utf-8'))

        elif self.path == '/api/mission/stop':

            if mission_thread and mission_thread.is_alive():
                if current_runner:
                    current_runner.stop = True
                mission_thread.join(timeout=3)

            # reply to the browser
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({'Stop'}).encode('utf-8'))

        elif self.path == '/api/mission/set?taskset=1':

            # Taskset 1
            TASKS = mission_all_v1_0.TASKS
            TOTAL_TIME = mission_all_v1_0.TOTAL_TIME
            GOAL_TIME_BUFFER = mission_all_v1_0.GOAL_TIME_BUFFER


            # reply to the browser
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({'taskset 1'}).encode('utf-8'))

        elif self.path == '/api/mission/set?taskset=2':
            # Taskset 2


            # reply to the browser
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({'taskset 2'}).encode('utf-8'))

        elif self.path == '/api/mission/set?taskset=3':
            # Taskset 3


            # reply to the browser
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({'taskset 3'}).encode('utf-8'))

        elif self.path == '/api/iwo/data':
            response = {
                'data': {
                    'robot_x': iwo.fused_x,           # in meters                    
                    'robot_y': iwo.fused_y,           # in meters
                    'robot_z': iwo.fused_z,           # in meters
                    # 'robot_roll': iwo.fused_roll,   
                    # 'robot_pitch': iwo.fused_pitch, # convert degrees to radians
                    # 'robot_yaw': iwo.fused_yaw      # convert degrees to radians


                    'robot_roll': np.radians(iwo.fused_roll),   # convert degrees to radians
                    'robot_pitch': np.radians(iwo.fused_pitch), # convert degrees to radians
                    'robot_yaw': np.radians(iwo.fused_yaw)      # convert degrees to radians
                    # 'robot_x': iwo_data.get('fused_x', 0),           # in meters
                    # 'robot_y': iwo_data.get('fused_y', 0),           # in meters
                    # 'robot_z': iwo_data.get('fused_z', 0),           # in meters
                    # 'robot_roll': np.radians(iwo_data.get('fused_roll', 0)),   # convert degrees to radians
                    # 'robot_pitch': np.radians(iwo_data.get('fused_pitch', 0)), # convert degrees to radians
                    # 'robot_yaw': np.radians(iwo_data.get('fused_yaw', 0))      # convert degrees to radians
                }
            }
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            # self.wfile.write(json.dumps(response).encode('utf-8'))
            self.wfile.write(json.dumps(response['data']).encode('utf-8'))

        else:
            try:
                import mimetypes
                req_path = self.path.split('?')[0].lstrip('/')
                if '..' in req_path:
                    self.send_error(403)
                    return
                curr_dir = os.path.dirname(os.path.abspath(__file__))
                filepath = os.path.join(curr_dir, req_path)
                if os.path.isfile(filepath):
                    mimetype, _ = mimetypes.guess_type(filepath)
                    with open(filepath, 'rb') as f:
                        file_content = f.read()
                    self.send_response(200)
                    self.send_header('Content-Type', mimetype or 'application/octet-stream')
                    self.send_header('Content-Length', len(file_content))
                    self.end_headers()
                    self.wfile.write(file_content)
                else:
                    self.send_error(404)
            except Exception as e:
                self.send_error(500, f"Error: {e}")


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

main_output = stream_manager.add_stream("main")
cameratest_output = stream_manager.add_stream("cameratest")
line_vision_stream_output = stream_manager.add_stream("line_vision")

def process_frames():
    global latest_frame
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(main={"size": (820, 616)},controls={'FrameDurationLimits': (50000, 50000)}))
    picam2.start()
    # config = picam2.create_video_configuration(
    #         main={"size": (820, 616), "format": "BGR888"}, 
    #         controls={
    #             "ExposureTime": 15000, 
    #             "AnalogueGain": 4.0, 
    #             "FrameDurationLimits": (33333, 33333)
    #         }
    # )
    # picam2.configure(config)
    # picam2.start()
    
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
    
    # Use centralized processor
    processor = aruco.get_processor()

    while True:
        if not aruco_enabled:
            time.sleep(0.1)
            continue
            
        with latest_frame_cond:
            latest_frame_cond.wait(timeout=0.2)
            if latest_frame is None:
                continue
            frame = latest_frame.copy()
            
        # Delegated everything to shared library!
        annotated_frame, data = processor.process_image(frame)
        aruco_data = data
        
        # Push annotated frame to cameratest_output
        jpeg_bw = simplejpeg.encode_jpeg(annotated_frame, quality=80, colorspace="RGB")
        cameratest_output.write(jpeg_bw)

def iwo_worker():
    global iwo_data
    while True:
        # Fetch IMU data (replace with actual IMU reading)
        # Example dummy data; integrate real IMU calls here
        x, y, z = 0, 0, 0  # Position (mm, if available; IMU typically gives orientation)
        yaw, pitch, roll = 0, 0, 0  # Orientation in radians

        x = iwo.fused_x
        y = iwo.fused_y
        z = iwo.fused_z
        yaw = iwo.fused_yaw
        pitch = iwo.fused_pitch
        roll = iwo.fused_roll

        iwo_data = [{
            'robot_x': x,
            'robot_y': y,
            'robot_z': z,
            'robot_yaw': yaw,
            'robot_pitch': pitch,
            'robot_roll': roll
        }]
        time.sleep(0.1)  # Poll rate, adjust as needed


# ADD TO THIS FUNCTION TO SPAWN MORE THREADS
def start_stream_server():
    import subprocess
    mqtt_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "mqtt_python")
    subprocess.Popen(["python3", "navigation.py"], cwd=mqtt_dir)
    threading.Thread(target=process_frames, daemon=True).start()
    threading.Thread(target=aruco_worker, daemon=True).start()
    threading.Thread(target=iwo_worker, daemon=True).start()

    # service.setup('localhost') # spawns its own rx and tx threads

    # gpio.setup()
    # robot.setup()
    # ir.setup()
    # pose.setup()
    # imu.setup()
    # cam.setup()
    # edge.setup()

    # iwo.setup()

    address = ('', 7123)
    server = StreamingServer(address, StreamingHandler)
    server.serve_forever()


if __name__ == "__main__":
    try:
        start_stream_server()
    finally:
        pass
