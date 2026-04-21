import time as t
import numpy as np
import cv2 as cv
from datetime import *
from setproctitle import setproctitle
import signal
from skimage.morphology import skeletonize

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
from utils import JunctionDirection, WindowDirection

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

def line_follow_vision(junctions = 'straight',
                       start_speed=0,
                       nominal_speed=0.2,
                       sensitivity=0.0025,
                       stop_speed=0,
                       time_to_full_speed=0,
                       stop_time=0,
                       timeout=None,
                       min_pixels_to_detect_line=-1):
    #print("Starting camera test stream...")

    # 1. Connect to the robot's main camera stream
    print("Connecting to main camera stream at localhost:7123...")
    cap = cv.VideoCapture("http://localhost:7123/stream/main")

    stop_event = threading.Event()

    # THIS ONLY WORKS IN THE MAIN THREAD, SO IT NEEDS TO BE DEACTIVATED WHEN RUNNING THE MISSION_RUNNER (THERE IS ALSO A PART BELOW THAT NEEDS TO BE UNCOMMENTED)
    # def my_signal_handler(sig, frame):
    #     print('UService:: You pressed Ctrl+C!')
    #     service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
    #     stop_event.set()
    # signal.signal(signal.SIGINT, my_signal_handler)

    # Start the mini web server on port 7124
    address = ('0.0.0.0', 7124)
    print("Starting OpenCV test stream on port 7124...")
    httpd = ThreadedHTTPServer(address, TestStreamHandler)
    
    def process_and_stream():
        global latest_jpeg
        failed_recognitions = 0
        MIN_PIXELS_TO_DETECT_LINE = min_pixels_to_detect_line if min_pixels_to_detect_line >= 0 else 10
        MAX_FAILED_RECOGNITIONS = 3
        speed = start_speed
        start_time = t.time()

        try:
            while not stop_event.is_set():
                ret, frame = cap.read()
                if ret and frame is not None:
                    # --- YOUR VISION CODE GOES HERE ---
                    t0 = t.time()
                    #image, heading = process_frame(frame, percentage_width=80)
                    #vision_steer_robot(heading, forward_speed=0.6, turn_sensitivity=0.005)
                    # vision_steer_robot(heading, forward_speed=speed, turn_sensitivity=sensitivity)
                    heading = None
                    if junctions == 'straight':
                        image, heading = process_frame(frame, MIN_PIXELS_TO_DETECT_LINE, percentage_width=80, percentage_height=50)
                    elif junctions == 'left':
                        image, heading = process_frame2(frame, 'left', MIN_PIXELS_TO_DETECT_LINE, percentage_width=80, percentage_height=50)
                    elif junctions == 'right':
                        image, heading = process_frame2(frame, 'right', MIN_PIXELS_TO_DETECT_LINE, percentage_width=80, percentage_height=50)

                    #image, heading = process_frame2(frame, 'left', percentage_width=80, percentage_height=40)
                    if heading is None:
                        failed_recognitions += 1
                        print(failed_recognitions)
                        if failed_recognitions >= MAX_FAILED_RECOGNITIONS:
                            print("Warning: heading was None, stopping vision")
                            if stop_time <= 0:
                                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                            else:
                                slow_down(stop_time, speed, stop_speed)
                            stop_event.set()
                            threading.Thread(target=httpd.shutdown, daemon=True).start()
                            break   # IMPORTANT: exit immediately
                    else:
                        #vision_steer_robot(heading, forward_speed=0.4, turn_sensitivity=0.005)
                        current_time = t.time()
                        failed_recognitions = 0

                        if timeout is not None and current_time - start_time > timeout - stop_time:
                            print("timeout is approaching, reducing speed...")
                            remaining_time = timeout - (current_time - start_time)
                            factor = max(0, min(1, remaining_time / stop_time))
                            speed = stop_speed + (nominal_speed - stop_speed) * factor
                            print(f"timeout is approaching, reducing speed... {speed}")
                            vision_steer_robot(heading, forward_speed=speed, turn_sensitivity=0.005+(sensitivity-0.005)*speed/nominal_speed)
                        else:
                            speed = start_speed + (nominal_speed-start_speed) * min(time_to_full_speed, t.time()-start_time) / time_to_full_speed
                            vision_steer_robot(heading, forward_speed=speed, turn_sensitivity=sensitivity)
                    #print(t.time()-t0)

                    # 2. Encode to JPEG (Note: OpenCV uses BGR colorspace by default!)
                    thresh_bgr = image
                    jpeg = simplejpeg.encode_jpeg(thresh_bgr, quality=80, colorspace='BGR')

                    # 3. Share the frame with the web server
                    with frame_condition:
                        latest_jpeg = jpeg
                        frame_condition.notify_all()
                else:
                    print("Warning: Failed to grab frame from main stream")
                # current_time = t.perf_counter()
                # print(f"Elapsed time: {current_time - prev_time:.6f}")
                # prev_time = current_time
                t.sleep(0.02) # this period should always be lower than the fps of the camera, otherwise the controller will work on older frames
        finally:
            print("Thread stopping robot")
            if stop_speed <= 0:
                print("Stopping Robot")
                t.sleep(5)
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
            

    # 🔹 Start worker thread
    worker_thread = threading.Thread(target=process_and_stream, daemon=True)
    worker_thread.start()

    try:
        httpd.serve_forever()
        
    # except KeyboardInterrupt:
    #     print("\nShutting down test stream.")
    #     httpd.server_close()
    #     service.send("robobot/cmd/ti", "rc 0.0 0.0")
    #     stop_event.set()
    finally:
        print("HTTP server stopped")
        # if stop_time > 0:
        #     print("Slowing Down Robot")
        #     slow_down(stop_time, nominal_speed, stop_speed)
        if stop_speed <= 0:
                #t.sleep(5)
                print("Stopping Robot")
                service.send("robobot/cmd/ti", "rc 0.0 0.0")

        stop_event.set()
        worker_thread.join()

        httpd.server_close()
        print("Vision fully stopped")

def slow_down(stop_time, speed, stop_speed):
    start_time = t.time()

    while True:
        elapsed = t.time() - start_time
        alpha = max(0.0, stop_time - elapsed / stop_time)

        v = stop_speed + (speed-stop_speed) * alpha
        service.send("robobot/cmd/ti", f"rc {v:.3f} 0.0")

        if alpha <= 0.0:
            break

        t.sleep(0.02)  # keep update rate similar to control loop


def drive_to_line():
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
        # prev_time = t.perf_counter()

        try:
            while not stop_event.is_set():
                ret, frame = cap.read()
                if ret and frame is not None:
                    # --- YOUR VISION CODE GOES HERE ---
                    image, heading = process_frame2(frame, percentage_height = 75, percentage_width=10)
                    #vision_steer_robot(heading)

                    # 2. Encode to JPEG (Note: OpenCV uses BGR colorspace by default!)
                    thresh_bgr = image
                    jpeg = simplejpeg.encode_jpeg(thresh_bgr, quality=80, colorspace='BGR')

                    # 3. Share the frame with the web server
                    with frame_condition:
                        latest_jpeg = jpeg
                        frame_condition.notify_all()
                else:
                    print("Warning: Failed to grab frame from main stream")
                # current_time = t.perf_counter()
                # print(f"Elapsed time: {current_time - prev_time:.6f}")
                # prev_time = current_time
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


def process_frame(img, MIN_PIXELS_TO_DETECT_LINE, percentage_height=40, percentage_width=100):
    # Choose relevant portion of the image and convert to grayscale
    h, w = img.shape[:2]
    h_relevant = (h * percentage_height) // 100
    w_relevant = (w * percentage_width) // 100
    x_start = (w - w_relevant) // 2

    roi = img[-h_relevant:, x_start:x_start + w_relevant]
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

    # Detect the line
    indices = np.arange(frame.shape[1])
    weights = clean / 255  # 0 or 1
    sums = np.sum(weights, axis=1)
    valid_rows = sums > 0  # Ignore rows with no white pixels

    if len(valid_rows) < MIN_PIXELS_TO_DETECT_LINE:
        heading = 0  # no line detected
    else:
        row_centers = np.sum(weights[valid_rows] * indices, axis=1) / sums[valid_rows]
        valid_indices = np.arange(h_relevant)[valid_rows]
        # row_weights = 1.5 * (valid_indices / h_relevant) + 0.5 # works for low speed
        row_weights = -2 * (valid_indices / h_relevant) + 2
        heading = np.sum((row_centers - w_relevant/2) * row_weights) / np.sum(row_weights)

        # Draw smooth continuous red line along the center
        points = [(int(row_centers[i]), valid_indices[i]) for i in range(len(valid_indices))]
        for i in range(1, len(points)):
            cv.line(frame, points[i-1], points[i], (0, 0, 255), 2)  # thickness=2

    return frame, heading


def process_frame2(img, follow, MIN_PIXELS_TO_DETECT_LINE, percentage_height=40, percentage_width=100):

    # Choose relevant portion of the image and convert to grayscale
    h, w = img.shape[:2]
    h_relevant = (h * percentage_height) // 100
    w_relevant = (w * percentage_width) // 100
    x_start = (w - w_relevant) // 2

    roi = img[-h_relevant:, x_start:x_start + w_relevant]
    gray = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
    _ , frame_gray = cv.threshold(gray, 160, 255, cv.THRESH_BINARY)

    # Only keep large areas
    num_labels, labels, stats, _ = cv.connectedComponentsWithStats(frame_gray)
    clean = np.zeros_like(frame_gray)
    # for i in range(1, num_labels):  # skip background
    #     area = stats[i, cv.CC_STAT_AREA]
    #     if area > 200:
    #         clean[labels == i] = 255
    
    if num_labels > 1:
        # skip background (index 0)
        areas = stats[1:, cv.CC_STAT_AREA]
        # find largest component index (add 1 because we skipped background)
        largest_label = 1 + np.argmax(areas)
        # keep only that component
        clean[labels == largest_label] = 255

    # --- Step 1: Contours ---
    contours, _ = cv.findContours(clean, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    # --- Step 2: Create edge image (only contour pixels) ---
    edge_img = np.zeros_like(clean)
    for cnt in contours:
        for pt in cnt:
            x, y = pt[0]
            edge_img[y, x] = 255

    # --- Step 3: Remove border margin ---
    margin = 10
    h2, w2 = edge_img.shape

    edge_img[:margin, :] = 0
    edge_img[-margin:, :] = 0
    edge_img[:, :margin] = 0
    edge_img[:, -margin:] = 0

    # --- Step 4: Connected components on edge image ---
    num_labels2, labels2, stats2, centroids2 = cv.connectedComponentsWithStats(edge_img)

    # --- Step 5: Select best cluster ---
    best_label = None
    best_score = -1000

    for i in range(1, num_labels2): # start at 1 because 0 is the black background (we don't want that one)
        area = stats2[i, cv.CC_STAT_AREA]
        cx, cy = centroids2[i]

        if area < 20:
            continue

        # Base score: prefer lower (closer to robot)
        #score = cy
        score = 0

        if follow == 'left':
            score -= cx * 0.5
        elif follow == 'right':
            score += cx * 0.5

        if score > best_score:
            best_score = score
            best_label = i

    # --- Step 6: Create filtered result ---
    filtered = np.zeros_like(edge_img)
    if best_label is not None:
        filtered[labels2 == best_label] = 255

    # Compute heading
    ys, xs = np.where(filtered == 255)

    heading = 0

    if len(xs) > MIN_PIXELS_TO_DETECT_LINE:
        # normalize row indices (same idea as your valid_indices / h_relevant)
        norm_y = ys / h2

        # same weighting function as your original code
        row_weights = -2 * norm_y + 2

        # horizontal error from center
        x_error = xs - (w2 / 2)

        # weighted sum over ALL pixels (not per-row)
        heading = np.sum(x_error * row_weights) / np.sum(row_weights)
    
    else:
        heading = None

    # --- Visualization ---
    output = cv.cvtColor(clean, cv.COLOR_GRAY2BGR)

    # Draw all contours in red
    cv.drawContours(output, contours, -1, (0, 0, 255), 1)

    # Draw selected cluster in green
    ys, xs = np.where(filtered == 255)
    for (x, y) in zip(xs, ys):
        cv.circle(output, (x, y), 1, (0, 255, 0), -1)

    return output, heading

def vision_steer_robot(heading, forward_speed=0.2, turn_sensitivity=0.0025):
    turn_rate = -heading * turn_sensitivity
    service.send("robobot/cmd/ti/","rc {} {}".format(forward_speed, turn_rate)) # (forward m/s, turn-rate rad/sec)