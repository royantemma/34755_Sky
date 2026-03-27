import time as t
import numpy as np
import cv2 as cv
from datetime import *
from setproctitle import setproctitle
import signal

import threading
import simplejpeg

from http import server
import socketserver


# robot function
from spose import pose
from sir import ir
from srobot import robot
from scam import cam
from sedge import edge
from sgpio import gpio
from scam import cam
from uservice import service
from lineTest import sky

import cv2 as cv
import threading
import simplejpeg
import time as t
from http import server
import socketserver

# Global variables to share the frame between the processing loop and the web server
latest_jpeg = None
frame_condition = threading.Condition()

class TestStreamHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
      # This will show up in your terminal to tell us EXACTLY what the browser wants
      print(f"DEBUG: Browser is asking for: '{self.path}'")

      # 1. If you just go to http://<ip>:7124/
      if self.path == '/' or self.path == '/index.html':
          content = b"<html><body><h1>Test Server Active</h1><img src='/stream'></body></html>"
          self.send_response(200)
          self.send_header('Content-Type', 'text/html')
          self.send_header('Content-Length', len(content))
          self.end_headers()
          self.wfile.write(content)

      # 2. If the browser (or the main page) asks for the video data
      elif '/stream' in self.path:
          self.send_response(200)
          self.send_header('Age', 0)
          self.send_header('Cache-Control', 'no-cache, private')
          self.send_header('Pragma', 'no-cache')
          self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
          self.end_headers()
          try:
              while True:
                  with frame_condition:
                      frame_condition.wait()
                      frame = latest_jpeg
                  
                  if frame is None: continue
                      
                  self.wfile.write(b'--FRAME\r\n')
                  self.send_header('Content-Type', 'image/jpeg')
                  self.send_header('Content-Length', len(frame))
                  self.end_headers()
                  self.wfile.write(frame)
                  self.wfile.write(b'\r\n')
          except Exception as e:
              print(f"Connection closed: {e}")

      # 3. If it's anything else, send the 404
      else:
          self.send_error(404, "File Not Found")

class ThreadedHTTPServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

def line_follow_vision():
    #print("Starting camera test stream...")
    # 1. Connect to the robot's main camera stream
    print("Connecting to main camera stream at localhost:7123...")
    cap = cv.VideoCapture("http://localhost:7123/stream/main")
    
    def process_and_stream():
      global latest_jpeg
      print("Checking main stream connection...")
      if not cap.isOpened():
        print("ERROR: Could not open the main stream at localhost:7123")
        return

      while True:
        ret, frame = cap.read()
        if ret and frame is not None:
            # --- YOUR VISION CODE GOES HERE ---
            image, heading = process_frame(frame)
            vision_steer_robot(heading)

            # 2. Encode to JPEG (Note: OpenCV uses BGR colorspace by default!)
            thresh_bgr = cv.cvtColor(image, cv.COLOR_GRAY2BGR)
            jpeg = simplejpeg.encode_jpeg(thresh_bgr, quality=80, colorspace='BGR')

            # 3. Share the frame with the web server
            with frame_condition:
                latest_jpeg = jpeg
                frame_condition.notify_all()
        else:
            print("Warning: Failed to grab frame from main stream")
        t.sleep(0.05)

    # Start the OpenCV processing in the background
    threading.Thread(target=process_and_stream, daemon=True).start()

    # Start the mini web server on port 7125
    address = ('0.0.0.0', 7124)
    print("Starting OpenCV test stream on port 7125...")
    httpd = ThreadedHTTPServer(address, TestStreamHandler)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down test stream.")
        httpd.server_close()


def process_frame(img, percentage_height=40, percentage_width=100):
    # Choose relevant portion of the image and convert to grayscale
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    h, w = gray.shape[:2]
    h_relevant = (h * percentage_height) // 100
    w_relevant = (w * percentage_width) // 100
    x_start = (w - w_relevant) // 2
    roi = gray[-h_relevant:, x_start:x_start + w_relevant]
    _, frame = cv.threshold(roi, 190, 255, cv.THRESH_BINARY)

    # Implement here dilation and erosion if needed

    # Detect the line
    indices = np.arange(frame.shape[1])
    weights = frame / 255  # 0 or 1
    sums = np.sum(weights, axis=1)
    valid_rows = sums > 0  # Ignore rows with no white pixels

    if not np.any(valid_rows):
        heading = 0  # no line detected
    else:
        row_centers = np.sum(weights[valid_rows] * indices, axis=1) / sums[valid_rows]
        valid_indices = np.arange(h_relevant)[valid_rows]
        row_weights = 1.5 * (valid_indices / h_relevant) + 0.5
        heading = np.sum((row_centers - w_relevant/2) * row_weights) / np.sum(row_weights)

        # Draw smooth continuous red line along the center
        points = [(int(row_centers[i]), valid_indices[i]) for i in range(len(valid_indices))]
        for i in range(1, len(points)):
            cv.line(color_roi, points[i-1], points[i], (0, 0, 255), 2)  # thickness=2

    return frame, heading


def vision_steer_robot(heading, forward_speed=0.2, turn_sensitivity=0.005):
    turn_rate = -heading * turn_sensitivity
    service.send("robobot/cmd/ti/","rc {} {}".format(forward_speed, turn_rate)) # (forward m/s, turn-rate rad/sec)