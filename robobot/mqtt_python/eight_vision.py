import time as t
import numpy as np
import cv2 as cv
from datetime import *
from setproctitle import setproctitle
import signal
from sfuse import iwo

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

def wait_for_police(percentage_height, time_between_checks, min_distance_for_movement, min_free_frames_to_move, require_movement_before_start, percentage_from_bottom): 

    # 1. Connect to the robot's main camera stream
    print("Connecting to main camera stream at localhost:7123...")
    cap = cv.VideoCapture("http://localhost:7123/stream/main")

    stop_event = threading.Event()

    # Start the mini web server on port 7124
    address = ('0.0.0.0', 7124)
    print("Starting OpenCV test stream on port 7124...")
    httpd = ThreadedHTTPServer(address, TestStreamHandler)


    # THIS ONLY WORKS IN THE MAIN THREAD, SO IT NEEDS TO BE DEACTIVATED WHEN RUNNING THE MISSION_RUNNER (THERE IS ALSO A PART BELOW THAT NEEDS TO BE UNCOMMENTED)
    # def my_signal_handler(sig, frame):
    #     print('UService:: You pressed Ctrl+C!')
    #     service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
    #     stop_event.set()
    # signal.signal(signal.SIGINT, my_signal_handler)
    
    def process_and_stream():
        global latest_jpeg
        MIN_DISTANCE_FOR_MOVEMENT = min_distance_for_movement
        MIN_FREE_FRAMES_TO_MOVE = min_free_frames_to_move

        start_time = t.time()
        t1 = t.time()
        prev_time = start_time
        centers = []
        counter_free_frames = 0
        movement_detected = False
        

        try:
            while not stop_event.is_set():
                ret, frame = cap.read()
                if ret and frame is not None:
                    # ROBOT is in front of the eight (next to the red extra time) and await for a free entry to the eight
                    if t.time() - prev_time >= time_between_checks: # only get 5fps for clear movement detection
                        prev_time = t.time()
                        image, centers = process_frame(frame, centers, percentage_width=100, percentage_height=percentage_height, percentage_from_bottom=percentage_from_bottom)
                        # Check for movement
                        if len(centers) > 1:
                            if np.linalg.norm(np.subtract(centers[-1], centers[-2])) >= MIN_DISTANCE_FOR_MOVEMENT:
                                #print("Movement Detected")
                                counter_free_frames = 0
                                movement_detected = True
                            else:
                                #print("Clear Space Detected")
                                if require_movement_before_start and movement_detected:
                                    counter_free_frames += 1
                                elif not require_movement_before_start:
                                    counter_free_frames += 1
                        
                        if counter_free_frames >= MIN_FREE_FRAMES_TO_MOVE:
                            stop_event.set()
                            threading.Thread(target=httpd.shutdown, daemon=True).start()
                            break   # IMPORTANT: exit immediately

                        # 2. Encode to JPEG (Note: OpenCV uses BGR colorspace by default!)
                        jpeg = simplejpeg.encode_jpeg(image, quality=80, colorspace='BGR')

                        # 3. Share the frame with the web server
                        with frame_condition:
                            latest_jpeg = jpeg
                            frame_condition.notify_all()
                else:
                    print("Warning: Failed to grab frame from main stream")
                #print("delay with previous frame" + str(t.time() - t1))
                t1 = t.time()
                sleep_time = min(t.time()-t1, 0.02)
                t.sleep(sleep_time) # this period should always be lower than the fps of the camera, otherwise the controller will work on older frames
        finally:
            print("Thread stopping robot")
            service.send("robobot/cmd/ti", "rc 0.0 0.0")

    # Start the OpenCV processing in the background
    worker_thread = threading.Thread(target=process_and_stream, daemon=True)
    worker_thread.start()

    
    try:
        httpd.serve_forever()
    finally:
        #print("i love breaking stuff...")
        stop_event.set()
        worker_thread.join()
        httpd.server_close()
        print("HTTP server stopped")
        print("Vision fully stopped")

        


def process_frame(img, prev_centers, percentage_height=60, percentage_width=100, percentage_from_bottom=0):
    # Choose relevant portion of the image and convert to grayscale
    h, w = img.shape[:2]
    h_relevant = (h * percentage_height) // 100
    h_start = h * percentage_from_bottom // 100
    
    roi = img[h-h_relevant:h-h_start, :]
    gray = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
    _ , frame_gray = cv.threshold(gray, 160, 255, cv.THRESH_BINARY)
    # frame_gray = cv.adaptiveThreshold(gray, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY, 11, 10)

    # Only keep large areas
    num_labels, labels, stats, _ = cv.connectedComponentsWithStats(frame_gray)
    clean = np.zeros_like(frame_gray)
    for i in range(1, num_labels):  # skip background
        area = stats[i, cv.CC_STAT_AREA]
        if area > 200:
            clean[labels == i] = 255
    frame = cv.cvtColor(clean, cv.COLOR_GRAY2BGR)

    # --- Compute center of white pixels ---
    ys, xs = np.where(clean == 255)
    centers = prev_centers.copy()

    if len(xs) > 0:
        cx = int(np.mean(xs))
        cy = int(np.mean(ys))
        centers.append([cx, cy])

    # --- Draw centers ---
    for (x, y) in centers:
        frame = cv.circle(frame, (x, y), 3, (0, 0, 255), -1)  # red dot

    return frame, centers
