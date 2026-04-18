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

def drive_roundabout():
    #print("Starting camera test stream...")

    # 1. Connect to the robot's main camera stream
    print("Connecting to main camera stream at localhost:7123...")
    cap = cv.VideoCapture("http://localhost:7123/stream/main")

    stop_event = threading.Event()
    def my_signal_handler(sig, frame):
        print('UService:: You pressed Ctrl+C!')
        service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
        stop_event.set()
    signal.signal(signal.SIGINT, my_signal_handler)
    
    def process_and_stream():
        global latest_jpeg
        error = 0

        try:
            while not stop_event.is_set():
                ret, frame = cap.read()
                if ret and frame is not None:
                    # --- YOUR VISION CODE GOES HERE ---
                    image, heading = process_frame(frame, percentage_width=80)
                    error += heading
                    # print(heading)
                    vision_steer_robot(heading, error, forward_speed=0.25, Kp=5, Ki=0.1)

                    # 2. Encode to JPEG (Note: OpenCV uses BGR colorspace by default!)
                    jpeg = simplejpeg.encode_jpeg(image, quality=80, colorspace='BGR')

                    # 3. Share the frame with the web server
                    with frame_condition:
                        latest_jpeg = jpeg
                        frame_condition.notify_all()
                else:
                    print("Warning: Failed to grab frame from main stream")
                t.sleep(0.02) # this period should always be lower than the fps of the camera, otherwise the controller will work on older frames
        finally:
            print("Thread stopping robot")
            service.send("robobot/cmd/ti", "rc 0.0 0.0")

    # Start the OpenCV processing in the background
    threading.Thread(target=process_and_stream, daemon=True).start()

    # Start the mini web server on port 7124
    address = ('0.0.0.0', 7124)
    print("Starting OpenCV test stream on port 7124...")
    httpd = ThreadedHTTPServer(address, TestStreamHandler)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down test stream.")
        httpd.server_close()
        service.send("robobot/cmd/ti", "rc 0.0 0.0")
        stop_event.set()


def process_frame(img, percentage_height=60, percentage_width=100):
    # Choose relevant portion of the image and convert to grayscale
    h, w = img.shape[:2]
    h_relevant = (h * percentage_height) // 100
    w_relevant = (w * percentage_width) // 100
    x_start = (w - w_relevant) // 2

    roi = img[-h_relevant:, x_start:x_start + w_relevant]
    gray = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
    adaptive_thresh = cv.adaptiveThreshold(
        gray,
        255,  # max value
        cv.ADAPTIVE_THRESH_GAUSSIAN_C,  # method (or MEAN_C)
        cv.THRESH_BINARY,
        11,  # block size (must be odd)
        3    # constant subtracted (lower value necessary for higher speeds?)
    )

    # Aggressive closing
    kernel = np.ones((5, 5), np.uint8)
    eroded = cv.erode(adaptive_thresh, kernel, iterations=6)

    # Process the image
    white_pixels = np.sum(eroded == 255)
    total_pixels = eroded.size
    ratio = white_pixels / total_pixels
    heading = ratio - 0.6

    frame = cv.cvtColor(eroded, cv.COLOR_GRAY2BGR)

    return frame, heading


def vision_steer_robot(heading, error, forward_speed=0.2, Kp = 8.5, Ki = 1):
    turn_rate = -heading * Kp - error * Ki
    service.send("robobot/cmd/ti/","rc {} {}".format(forward_speed, turn_rate)) # (forward m/s, turn-rate rad/sec)