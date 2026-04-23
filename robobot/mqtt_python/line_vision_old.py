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

def line_follow_vision(junctions = [JunctionDirection.LEFT], speed=0.2, sensitivity=0.0025):
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
        junction_start_index = 0
        prev_position_intersections = []
        # prev_time = t.perf_counter()

        try:
            while not stop_event.is_set():
                ret, frame = cap.read()
                if ret and frame is not None:
                    # --- YOUR VISION CODE GOES HERE ---
                    t0 = t.time()
                    #image, heading = process_frame(frame, percentage_width=80)
                    #vision_steer_robot(heading, forward_speed=0.6, turn_sensitivity=0.005)
                    # vision_steer_robot(heading, forward_speed=speed, turn_sensitivity=sensitivity)
                    # clean_binary = process_frame2(frame, percentage_width=80)
                    # heading, image, position_intersections = get_heading(clean_binary, junctions, junction_start_index)
                    # junction_start_index = update_junction_index(prev_position_intersections, position_intersections, junction_start_index)
                    # prev_position_intersections = position_intersections
                    # print(t.time()-t0)
                    
                    image, heading = process_frame4(frame, 'left', percentage_width=80)
                    

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
                    vision_steer_robot(heading)

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


def process_frame(img, percentage_height=40, percentage_width=100):
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

    if not np.any(valid_rows):
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

def process_frame2(img, percentage_height=40, percentage_width=100):

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
    for i in range(1, num_labels):  # skip background
        area = stats[i, cv.CC_STAT_AREA]
        if area > 200:
            clean[labels == i] = 255
    return clean

def process_frame4(img, follow, percentage_height=40, percentage_width=100):

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
    for i in range(1, num_labels):  # skip background
        area = stats[i, cv.CC_STAT_AREA]
        if area > 200:
            clean[labels == i] = 255
    
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
        score = cy

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

    if len(xs) > 0:

        # normalize row indices (same idea as your valid_indices / h_relevant)
        norm_y = ys / h

        # same weighting function as your original code
        row_weights = -2 * norm_y + 2

        # horizontal error from center
        x_error = xs - (w / 2)

        # weighted sum over ALL pixels (not per-row)
        heading = np.sum(x_error * row_weights) / np.sum(row_weights)

    # --- Visualization ---
    output = cv.cvtColor(clean, cv.COLOR_GRAY2BGR)

    # Draw all contours in red
    cv.drawContours(output, contours, -1, (0, 0, 255), 1)

    # Draw selected cluster in green
    ys, xs = np.where(filtered == 255)
    for (x, y) in zip(xs, ys):
        cv.circle(output, (x, y), 1, (0, 255, 0), -1)

    return output, heading



    
def get_heading(clean_binary, junctions, start_junction_index):
    WINDOW_SHAPE = [120, 30] # [x, y]
    h, w = clean_binary.shape
    path = []
    junction_index = start_junction_index

    bottom_row = clean_binary[-1,:]
    x = get_center_first_row(bottom_row)
    y = h-1

    if x == -1: # --- SAFETY --- no white pixels detected in the bottom row
        for i in range(h-2,0,-1):
            row = clean_binary[i,:]
            x = get_center_first_row(row)
            if x == -1:
                break
            y -= 1
        print("WARNING: No white pixels detected in image. Heading set to zero")
        return 0 # return a heading of zero
    path.append([x, y])
    y -= 1

    # No detection check on the first row (bc I'm lazy)

    intersection_detected = False
    counter_no_intersection = 0
    POINTS_WITHOUT_INTERSECTION = 10 # number of points necessary for an intersection to be considered "over"
    position_intersections = []

    while 5 < x < w-5 and 5 < y < h: 

        if junctions[junction_index] == JunctionDirection.NONE or junctions[junction_index] == JunctionDirection.LEFT or junctions[junction_index] == JunctionDirection.RIGHT:
            window_orientation = WindowDirection.TOP
            window, ox, oy = get_window(clean_binary, x, y, WINDOW_SHAPE, window_orientation)
        elif junctions[junction_index] == JunctionDirection.SHARP_LEFT:
            window_orientation = WindowDirection.LEFT
            window, ox, oy = get_window(clean_binary, x, y, [WINDOW_SHAPE[1], WINDOW_SHAPE[0]], window_orientation)
        elif junctions[junction_index] == JunctionDirection.SHARP_RIGHT:
            window_orientation = WindowDirection.RIGHT
            window, ox, oy = get_window(clean_binary, x, y, [WINDOW_SHAPE[1], WINDOW_SHAPE[0]], WindowDirection.RIGHT)
        
        current_intersection_detected, cx, cy = analyse_window(window, window_orientation, junctions[junction_index])

        if current_intersection_detected:
            intersection_detected = True
        elif intersection_detected:
            counter_no_intersection += 1
            if counter_no_intersection > POINTS_WITHOUT_INTERSECTION:
                junction_index = min(len(junctions)-1, junction_index+1)
                counter_no_intersection = 0
                intersection_detected = False
        print(oy, cy)
        x = int(ox + cx)
        y = int(oy + cy)
        path.append([x, y])

        if current_intersection_detected and not intersection_detected:
            position_intersections.append([x, y])

    

    # compute heading
    xs = np.array([p[0] for p in path])
    weights = np.linspace(2, 0.5, len(xs))

    heading = np.sum((xs - w/2) * weights) / np.sum(weights)

    # Visualisation
    frame = cv.cvtColor(clean_binary, cv.COLOR_GRAY2BGR)

    for i in range(1, len(path)):
        cv.line(frame, path[i-1], path[i], (0, 0, 255), 2)

    # Draw rectangle
    x0, y0 = path[0]
    top_left = (int(x0 - WINDOW_SHAPE[0]//2), int(y0 - WINDOW_SHAPE[1]//2))
    bottom_right = (int(x0 + WINDOW_SHAPE[0]//2), int(y0 + WINDOW_SHAPE[1]//2))
    cv.rectangle(frame, top_left, bottom_right, (0, 255, 0), 2)

    return frame, heading, position_intersections
        

def update_junction_index(prev_position_intersections, position_intersections, junction_start_index):
    MAX_DISTANCE_INTERSECTIONS = 100 # max distance in pixels between the position of intersections between 2 frames
    junction_index = junction_start_index
    for px, py in prev_position_intersections:
        found = False
        for x, y in position_intersections:
            distance = np.hypot(px-x, py-y)
            if distance < MAX_DISTANCE_INTERSECTIONS:
                found = True
                break
        if not found:
            junction_index += 1
    return junction_index

                
        

def analyse_window(window, windowDirection, junctionDirection):

    intersection_detected = detect_intersection(window, windowDirection)

    if intersection_detected:
        if junctionDirection == JunctionDirection.SHARP_LEFT:
            if windowDirection == WindowDirection.TOP:
                cx = 0
                column = window[0,:]
                cy = np.mean(np.where(column > 0)[0]) 
            elif windowDirection == WindowDirection.LEFT:
                row = window[-1,:]
                cx = np.mean(np.where(row > 0)[0]) 
                cy = window.shape[0]
            elif windowDirection == WindowDirection.RIGHT:
                row = window[0,:]
                cx = np.mean(np.where(row > 0)[0]) 
                cy = 0
        
        elif junctionDirection == JunctionDirection.SHARP_RIGHT:
            if windowDirection == WindowDirection.TOP:
                cx = window.shape[1]
                column = window[-1,:]
                cy = np.mean(np.where(column > 0)[0])
            elif windowDirection == WindowDirection.LEFT:
                row = window[0,:]
                cx = np.mean(np.where(row > 0)[0]) 
                cy = 0
            elif windowDirection == WindowDirection.RIGHT:
                row = window[0,:]
                cx = np.mean(np.where(row > 0)[0]) 
                cy = window.shape[0]

        elif junctionDirection == JunctionDirection.LEFT:
            if windowDirection == WindowDirection.TOP:
                row = window[-1,:]
                cx = compute_top_left(row)
                cy = 0
            elif windowDirection == WindowDirection.LEFT:
                column = window[:, 0]
                cx = 0
                cy = compute_top_right(column) # right is right here because we are looking from the image coordinate frame
            elif windowDirection == WindowDirection.RIGHT:
                column = window[:, -1]
                cx = window.shape[1]
                cy = compute_top_left(column)

        elif junctionDirection == JunctionDirection.RIGHT:
            if windowDirection == WindowDirection.TOP:
                row = window[-1,:]
                cx = compute_top_right(row)
                cy = 0
            elif windowDirection == WindowDirection.LEFT:
                column = window[:, 0]
                cx = 0
                cy = compute_top_left(column) # right is right here because we are looking from the image coordinate frame
            elif windowDirection == WindowDirection.RIGHT:
                column = window[:, -1]
                cx = window.shape[1]
                cy = compute_top_right(column)

        # JunctionDirection.NONE is not implemented, please choose LEFT or RIGHT

    else: # no intersection detected
        row = window[-1, :]
        cx = compute_top_mean(row)
        cy = window[-1]

    return intersection_detected, cx, cy
                



def get_center_first_row(row):
    xs = np.where(row > 0)[0] # looks for white pixels in the row
    if len(xs) == 0:
        return -1 # no white pixels detected on bottom row

    # split into contiguous segments
    segments = []
    start = xs[0]
    prev = xs[0]

    for i in xs[1:]:
        if i == prev + 1:
            prev = i
        else:
            segments.append((start, prev))
            start = i
            prev = i
    segments.append((start, prev))

    largest = max(segments, key=lambda s: s[1] - s[0]) # pick largest segment (true "largest blob")
    x = (largest[0] + largest[1]) // 2 # computer center of row
    return x


def get_window(image, x_center, y_center, shape, windowDirection):
    #print(x_center, y_center, shape)
    w, h = image.shape
    if windowDirection == WindowDirection.TOP:
        x1 = int(max(0, x_center - shape[0]//2)) # left bottom
        x2 = int(min(w, x_center + shape[0]//2))
        y1 = int(max(0, y_center)) # left bottom
        y2 = int(min(h, y_center - shape[1]))
        #print(x1, x2, y2, y1, w, h)
        #print(image[y2:y1, x1:x2])
        return image[y2:y1, x1:x2], x1, y2
    elif windowDirection == WindowDirection.LEFT:
        y1 = int(max(0, y_center + shape[1]//2)) # left bottom
        y2 = int(min(h, y_center - shape[1]//2))
        x1 = int(max(0, x_center)) # left bottom
        x2 = int(min(w, x_center - shape[0]))
        return image[y2:y1, x2:x1], x2, y2
    elif windowDirection == WindowDirection.RIGHT:
        y1 = int(max(0, y_center + shape[1]//2)) # left bottom
        y2 = int(min(h, y_center - shape[1]//2))
        x1 = int(max(0, x_center)) # left bottom
        x2 = int(min(w, x_center + shape[0]))
        return image[y2:y1, x1:x2], x1, y2


def detect_intersection(window, windowDirection):
    print(window)
    SIDE_PIXELS_FOR_DETECTION = 7
    ALLOWED_BLACK_PIXELS_IN_LINE = 7

    if windowDirection == WindowDirection.TOP:
        left_edge = np.sum(window[:,0] > 0)
        right_edge = np.sum(window[:, -1] > 0)
        top_indices = np.where(window[0, :] > 0)[0]

    elif windowDirection == WindowDirection.LEFT:
        left_edge = np.sum(window[-1,:] > 0)
        right_edge = np.sum(window[0, :] > 0)
        top_indices = np.where(window[:, 0] > 0)[0]
    
    elif windowDirection == WindowDirection.RIGHT:
        left_edge = np.sum(window[0,:] > 0)
        right_edge = np.sum(window[-1, :] > 0)
        top_indices = np.where(window[:, -1] > 0)[0]
        
    if left_edge > SIDE_PIXELS_FOR_DETECTION or right_edge > SIDE_PIXELS_FOR_DETECTION:
        return True # intersection detected
    elif len(top_indices) > 0:
        for i in range(1, len(top_indices)):
            if top_indices[i] > top_indices[i - 1] + ALLOWED_BLACK_PIXELS_IN_LINE:
                return True # intersection detected
    
    return False # no intersection was detected


def compute_top_left(vector):
    s = np.where(vector > 0)[0]
    prev = s[0]

    segment = []
    for i in s[1:]:
        segment.append(prev)
        if i == prev + 1:
            prev = i
        else:
            break
    return np.mean(segment)


def compute_top_right(vector):
    s = np.where(vector > 0)[0]
    prev = s[-1]

    segment = []
    for i in s[-2::-1]:
        segment.append(prev)
        if i == prev - 1:
            prev = i
        else:
            break
    return np.mean(segment)

def compute_top_mean(vector):
    s = np.where(vector > 0)[0]
    return np.mean(s)




# def process_frame3(img, junction):

#     h2, w2 = clean.shape
#     # -------------------------
#     # helper functions
#     # -------------------------
#     def get_window(x, y, size=120):
#         half = size // 2
#         x1 = int(max(0, x - half))
#         x2 = int(min(w2, x + half))
#         y1 = int(max(0, y - half))
#         y2 = int(min(h2, y + half))
#         return clean[y1:y2, x1:x2], x1, y1

#     def detect_junction(window):
#         """
#         Returns JunctionType based on edge hits
#         """

#         h, w = window.shape
#         if h == 0 or w == 0:
#             return JunctionType.NONE

#         # -----------------------------------------
#         # SHARP_LEFT / SHARP_RIGHT detection
#         # -----------------------------------------
#         left_edge = np.sum(window[:, 0] > 0)
#         right_edge = np.sum(window[:, -1] > 0)

#         # thresholds (tunable)
#         if left_edge > 5:
#             return JunctionType.SHARP_LEFT
#         if right_edge > 5:
#             return JunctionType.SHARP_RIGHT

        
#         # -----------------------------------------
#         # TOP only if 2+ segments (Y-junction)
#         # -----------------------------------------
#         top_row = window[0, :] > 0
#         top_indices = np.where(top_row)[0]

#         num_top_segments = 0

#         if len(top_indices) > 0:
#             num_top_segments = 1
#             for i in range(1, len(top_indices)):
#                 if top_indices[i] != top_indices[i - 1] + 1:
#                     num_top_segments += 1
        
#         if num_top_segments >= 2:
#             if current_junction == JunctionType.LEFT:
#                 return JunctionType.LEFT
#             elif current_junction == JunctionType.RIGHT:
#                 return JunctionType.RIGHT
#             else:
#                 print("WARNING: TOP JUNCTION was detected but not expected")

#         return JunctionType.NONE

#     def row_center(window):
#         """Find horizontal center of white pixels in window"""
#         ys, xs = np.where(window > 0)
#         if len(xs) == 0:
#             return None
#         return int(np.mean(xs))

#     # -------------------------
#     # 2. start at bottom row
#     # -------------------------
#     y = h2 - 1
#     row = clean[y]

#     # find connected segments (blobs) in the bottom row to find the middle of the largest segment
#     xs = np.where(row > 0)[0]

#     if len(xs) == 0:
#         print("WARNING: no white edge detected on the bottom row")
#         return img, 0

#     # split into contiguous segments
#     segments = []
#     start = xs[0]
#     prev = xs[0]

#     for i in xs[1:]:
#         if i == prev + 1:
#             prev = i
#         else:
#             segments.append((start, prev))
#             start = i
#             prev = i
#     segments.append((start, prev))

#     # pick largest segment (true "largest blob")
#     largest = max(segments, key=lambda s: s[1] - s[0])

#     # center of largest blob
#     x = (largest[0] + largest[1]) // 2

#     path = [(int(x), int(y))]
#     heading_signal = []

#     # -------------------------
#     # 3. iterate upward
#     # -------------------------
#     window_size = 120
#     for _ in range(h2):

#         window, ox, oy = get_window(x, y, size=window_size)
#         row = window[window.shape[0] // 2]

#         detected_junction = detect_junction(window)
#         if detected_junction != JunctionType.NONE and detected_junction != current_junction:
#             print(f"Warning: junction {detected_junction} was detected while junction {current_junction} was expected")
#             detected_junction = JunctionType.NONE

#         segment = []

#         if detected_junction == JunctionType.LEFT:
#             # follow the most left segment of the line
#             xs = np.where(row > 0)[0]
#             prev = xs[0]

#             for i in xs[1:]:
#                 if i == prev + 1:
#                     segment.append(prev)
#                     prev = i
#                 else:
#                     segment.append(prev)
#             if segment:
#                 cx = np.mean(segment)
#             else:
#                 print("Warning: no white line detected for JunctionType.LEFT")
#                 cx = window_size//2
#             x = ox + cx

#         elif detected_junction == JunctionType.RIGHT:
#             # follow the most right segment of the line
#             xs = np.where(row > 0)[0]
#             prev = xs[-1]

#             for i in xs[-2::-1]:
#                 if i == prev - 1:
#                     segment.append(prev)
#                     prev = i
#                 else:
#                     segment.append(prev)
#             if segment:
#                 cx = np.mean(segment)
#             else:
#                 print("Warning: no white line detected for JunctionType.LEFT")
#                 cx = window_size//2
#             x = ox + cx

#         elif detected_junction == JunctionType.SHARP_LEFT:
#             # follow the segment on the left of the window
#             column = window[:,0]
#             ys = np.where(column > 0)[0]
#             prev = ys[0]

#             for i in ys[1:]:
#                 if i == prev + 1:
#                     segment.append(prev)
#                     prev = i
#                 else:
#                     segment.append(prev)
#             if segment:
#                 cy = np.mean(segment)
#             else:
#                 print("Warning: no white line detected for JunctionType.LEFT")
#                 cy = window_size//2
#             y = oy + cy

#         elif detected_junction == JunctionType.SHARP_RIGHT:
#             # follow the semgent on the right of the window
#             column = window[:,-1]
#             ys = np.where(column > 0)[0]
#             prev = ys[0]

#             for i in ys[1:]:
#                 if i == prev + 1:
#                     segment.append(prev)
#                     prev = i
#                 else:
#                     segment.append(prev)
#             if segment:
#                 cy = np.mean(segment)
#             else:
#                 print("Warning: no white line detected for JunctionType.LEFT")
#                 cy = window_size//2
#             y = oy + cy
#         else:
#             # follow the middle of the top edge
#             xs = np.where(row > 0)[0]
#             cx = np.mean(xs)
#             x = ox + cx

#         if detected_junction == JunctionType.SHARP_LEFT:
#             if x <=0:
#                 break
#         elif detected_junction == JunctionType.SHARP_RIGHT:
#             x += 1
#             if x >= w:
#                 break
#         else:
#             # move up one row
#             y -= 1
#             if y <= 0:
#                 break

#         path.append((int(x), int(y)))

#     # -------------------------
#     # 4. compute heading
#     # -------------------------
#     if len(path) < 2:
#         return img, 0

#     xs = np.array([p[0] for p in path])
#     weights = np.linspace(2, 0.5, len(xs))

#     heading = np.sum((xs - w2 / 2) * weights) / np.sum(weights)

#     # -------------------------
#     # 5. visualization
#     # -------------------------
#     frame = cv.cvtColor(clean, cv.COLOR_GRAY2BGR)

#     for i in range(1, len(path)):
#         cv.line(frame, path[i-1], path[i], (0, 0, 255), 2)

#     # Draw rectangle
#     x0, y0 = path[0]
#     half = window_size // 2 

#     top_left = (int(x0 - half), int(y0 - half))
#     bottom_right = (int(x0 + half), int(y0 + half))

#     cv.rectangle(frame, top_left, bottom_right, (0, 255, 0), 2)

#     return frame, heading


# def vision_steer_robot(heading, forward_speed=0.2, turn_sensitivity=0.0025):
#     turn_rate = -heading * turn_sensitivity
#     service.send("robobot/cmd/ti/","rc {} {}".format(forward_speed, turn_rate)) # (forward m/s, turn-rate rad/sec)