import time as t
import numpy as np
import cv2 as cv
from datetime import *
from setproctitle import setproctitle
import signal

import threading
import simplejpeg
from scam import getImage

from http import server
import socketserver

import math


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

"""
Insights into the working of the mqtt-client (Henrik)

# Code stucture and where we integrate
Our custom code goes into the loop() function. The code blow shows a blueprint for
such a mission (custom_mission).


### Hardware control and interfacing
# Motor control
service.send("robobot/cmd/ti","rc 0.2 0.5") # (forward m/s, turn-rate rad/sec) 
This sends the desired forward speed and turn rate to the robot. The CMixer on the teensy
then takes care of the rest.

# Control of LEDs
service.send("robobot/cmd/T0","leds 16 0 100 0") # green

"""

import cv2 as cv
import threading
import simplejpeg
import time as t
from http import server
import socketserver

# Global variables to share the frame between the processing loop and the web server
latest_jpeg = None
frame_condition = threading.Condition()

"""
def go_to_xy(x_goal, y_goal, yaw_goal=None, max_speed=0.8, dist_tolerance=0.02):
    yaw_goal = np.deg2rad(yaw_goal) if yaw_goal is not None else None
    # PID Gains
    Kp_lin = 0.4; Ki_lin = 0.0; Kd_lin = 0.02
    Kp_ang = 1.5; Ki_ang = 0.0; Kd_ang = 0.05
    
    from sfuse import iwo
    e_lin_int = 0; e_lin_prev = 0
    e_ang_int = 0; e_ang_prev = 0
    last_time = t.time()

    xy_reached = False

    while True:
        # Get current state
        curr_x, curr_y, curr_yaw = iwo.fused_x, iwo.fused_y, np.deg2rad(iwo.fused_yaw)
        dt = t.time() - last_time
        if dt <= 0: dt = 0.001 # Prevent division by zero

        if not xy_reached:
            # --- PHASE 1: Drive to XY ---
            dist_error = math.sqrt((x_goal - curr_x)**2 + (y_goal - curr_y)**2)
            desired_heading = math.atan2(y_goal - curr_y, x_goal - curr_x)
            angle_error = math.atan2(math.sin(desired_heading - curr_yaw), math.cos(desired_heading - curr_yaw))

            if dist_error < dist_tolerance: 
                print("XY Target reached!")
                if yaw_goal is None:
                    break # Exit completely if no yaw is requested
                xy_reached = True
                # Reset PID terms for the rotation phase
                e_ang_int = 0; e_ang_prev = 0 
                continue

            v = (Kp_lin * dist_error) + (Ki_lin * e_lin_int) + (Kd_lin * (dist_error - e_lin_prev) / dt)
            # Prioritize turning: don't drive forward if pointing the wrong way
            if abs(angle_error) > 0.5: v = 0 
            
            w = (Kp_ang * angle_error) + (Ki_ang * e_ang_int) + (Kd_ang * (angle_error - e_ang_prev) / dt)
            e_lin_prev = dist_error

        else:
            # --- PHASE 2: Reach Final Heading ---
            angle_error = math.atan2(math.sin(yaw_goal - curr_yaw), math.cos(yaw_goal - curr_yaw))
            
            if abs(angle_error) < 0.03: # Within ~1.7 degrees
                print("Final heading reached!")
                break

            v = 0 # No forward movement in phase 2
            w = (Kp_ang * angle_error) + (Ki_ang * e_ang_int) + (Kd_ang * (angle_error - e_ang_prev) / dt)
            e_ang_prev = angle_error

        # Speed Saturation & Command
        v = max(min(v, max_speed), -max_speed)
        w = max(min(w, 1.5), -1.5)
        service.send("robobot/cmd/ti", f"rc {v:.3f} {w:.3f}")

        last_time = t.time()
        t.sleep(0.05)

    # Final Stop
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
"""

def go_to_xy(x_goal, y_goal, yaw_goal=None, max_speed=0.8, dist_tolerance=0.02):
    from sfuse import iwo
    #from simu import imu

    yaw_goal = np.deg2rad(yaw_goal) if yaw_goal is not None else None

    # PID Gains
    Kp_lin = 0.4; Ki_lin = 0.0; Kd_lin = 0.02
    Kp_ang = 1.5; Ki_ang = 0.0; Kd_ang = 0.05

    e_lin_int = 0.0; e_lin_prev = 0.0
    e_ang_int = 0.0; e_ang_prev = 0.0
    last_time = t.time()

    xy_reached = False
    rot_phase_initialized = False  # for shortest-path yaw phase

    def shortest_yaw_error(curr_yaw, yaw_target):
        return math.atan2(math.sin(yaw_target - curr_yaw),
                          math.cos(yaw_target - curr_yaw))

    while True:
        # Get current state
        #print("Current Yaw: " + str(iwo.fused_yaw) )# ",# Gyro: " + str(imu.gyro))
        curr_x, curr_y, curr_yaw = iwo.fused_x, iwo.fused_y, np.deg2rad(iwo.fused_yaw)
        now = t.time()
        dt = now - last_time
        if dt <= 0: dt = 0.001

        if not xy_reached:
            # --- PHASE 1: Drive to XY ---
            dist_error = math.sqrt((x_goal - curr_x)**2 + (y_goal - curr_y)**2)
            desired_heading = math.atan2(y_goal - curr_y, x_goal - curr_x)
            angle_error = shortest_yaw_error(curr_yaw, desired_heading)

            if dist_error < dist_tolerance:
                print("XY Target reached!")
                if yaw_goal is None:
                    break
                xy_reached = True
                # Reset angular PID terms for rotation phase
                e_ang_int = 0.0; e_ang_prev = 0.0
                rot_phase_initialized = False
                last_time = now
                continue

            # Linear PID
            v = (Kp_lin * dist_error
                 + Ki_lin * e_lin_int
                 + Kd_lin * (dist_error - e_lin_prev) / dt)
            
            # Taken over from the fast method
            v = max(v, max_speed/6) if v >= 0 else min(v, -max_speed/6)
            # Prioritize turning
            if abs(angle_error) > 0.5:
                v = 0.0

            # Angular PID
            w = (Kp_ang * angle_error
                 + Ki_ang * e_ang_int
                 + Kd_ang * (angle_error - e_ang_prev) / dt)

            e_lin_prev = dist_error
            # (optional) e_lin_int += dist_error * dt

        else:
            # --- PHASE 2: Shortest‑path yaw rotation with continuous reference ---
            if not rot_phase_initialized:
                yaw_start = curr_yaw
                delta = shortest_yaw_error(yaw_start, yaw_goal)
                # Nominal rotation speed ~1 rad/s → duration:
                T_rot = max(0.5, abs(delta) / 1.0)
                t_start = now
                rot_phase_initialized = True

            # Progress 0→1
            s = (now - t_start) / T_rot
            if s > 1.0:
                s = 1.0

            # Smooth interpolation (cosine ease-in-out)
            s_smooth = 0.5 * (1.0 - math.cos(math.pi * s))

            # Continuous yaw reference
            yaw_ref = yaw_start + delta * s_smooth
            yaw_ref = math.atan2(math.sin(yaw_ref), math.cos(yaw_ref))

            angle_error = shortest_yaw_error(curr_yaw, yaw_ref)

            # Stop condition: close to final goal and reference finished
            final_err = shortest_yaw_error(curr_yaw, yaw_goal)
            if abs(final_err) < 0.03 and s >= 1.0:
                print("Final heading reached!")
                break

            v = 0.0  # no forward motion in rotation phase
            w = (Kp_ang * angle_error
                 + Ki_ang * e_ang_int
                 + Kd_ang * (angle_error - e_ang_prev) / dt)
            
            # Taken over from the fast method
            #w = max(w, 0.1) if w >= 0 else min(w, -0.1)
            w = max(w, 0.3) if w >= 0 else min(w, -0.3)
            
            e_ang_prev = angle_error
            # (optional) e_ang_int += angle_error * dt

        # Saturation
        v = max(min(v, max_speed), -max_speed)
        w = max(min(w, 1.5), -1.5)

        service.send("robobot/cmd/ti", f"rc {v:.3f} {w:.3f}")

        last_time = now
        t.sleep(0.05)
        #t.sleep(0.1)

    # Final Stop
    service.send("robobot/cmd/ti", "rc 0.0 0.0")


def go_to_xy_fast(x_goal, y_goal, yaw_goal=None, max_speed=0.8, dist_tolerance=0.02):
    yaw_goal = np.deg2rad(yaw_goal) if yaw_goal is not None else None
    # PID Gains
    Kp_lin = 0.4; Ki_lin = 0.0; Kd_lin = 0.02
    Kp_ang = 1.5; Ki_ang = 0.0; Kd_ang = 0.05
    
    from sfuse import iwo
    e_lin_int = 0; e_lin_prev = 0
    e_ang_int = 0; e_ang_prev = 0
    last_time = t.time()

    xy_reached = False

    while True:
        # Get current state
        curr_x, curr_y, curr_yaw = iwo.fused_x, iwo.fused_y, np.deg2rad(iwo.fused_yaw)
        dt = t.time() - last_time
        if dt <= 0: dt = 0.001 # Prevent division by zero

        if not xy_reached:
            # --- PHASE 1: Drive to XY ---
            dist_error = math.sqrt((x_goal - curr_x)**2 + (y_goal - curr_y)**2)
            desired_heading = math.atan2(y_goal - curr_y, x_goal - curr_x)
            angle_error = math.atan2(math.sin(desired_heading - curr_yaw), math.cos(desired_heading - curr_yaw))

            if dist_error < dist_tolerance: 
                print("XY Target reached!")
                if yaw_goal is None:
                    break # Exit completely if no yaw is requested
                xy_reached = True
                # Reset PID terms for the rotation phase
                e_ang_int = 0; e_ang_prev = 0 
                continue

            v = (Kp_lin * dist_error) + (Ki_lin * e_lin_int) + (Kd_lin * (dist_error - e_lin_prev) / dt)
            v = max(v, max_speed/6) if v >= 0 else min(v, -max_speed/6)
            # Prioritize turning: don't drive forward if pointing the wrong way
            if abs(angle_error) > 0.5: v = 0 
            
            w = (Kp_ang * angle_error) + (Ki_ang * e_ang_int) + (Kd_ang * (angle_error - e_ang_prev) / dt)
            e_lin_prev = dist_error

        else:
            # --- PHASE 2: Reach Final Heading ---
            angle_error = math.atan2(math.sin(yaw_goal - curr_yaw), math.cos(yaw_goal - curr_yaw))
            
            if abs(angle_error) < 0.03: # Within ~1.7 degrees
                print("Final heading reached!")
                break

            v = 0 # No forward movement in phase 2
            w = (Kp_ang * angle_error) + (Ki_ang * e_ang_int) + (Kd_ang * (angle_error - e_ang_prev) / dt)
            w = max(w, 0.1) if w >= 0 else min(w, -0.1)
            e_ang_prev = angle_error

        # Speed Saturation & Command
        v = max(min(v, max_speed), -max_speed)
        w = max(min(w, 1.5), -1.5)
        service.send("robobot/cmd/ti", f"rc {v:.3f} {w:.3f}")

        last_time = t.time()
        #t.sleep(0.1)
        t.sleep(0.05)

    # Final Stop
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
        


def test_xyyaw(x=5.41+0.28,y=-2.33+0.48,yaw=0):
  
  # Resetting Odometry
  state = 0
  pose.tripBreset()

  print("% Starting XY mission! -------------------------")
  service.send("robobot/cmd/T0","leds 16 0 100 0") # green

  # Main "State Machine of the Mission"
  while not (service.stop):
      if state == 0: # wait for start signal
          go_to_xy(x, y, yaw)
          state = 1
      if state == 1: # go back where you came from
          go_to_xy(0, 0, 0)
          state = 2
      else:
          print(f"# drive 1m drove {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds")
          service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
          break
  pass
  service.send("robobot/cmd/T0","leds 16 0 0 0") # end
  print("% Testing XY YAW ------------------------- end")
  
def test_BOB():
  print("Testing BOB")
  # Resetting Odometry
  state = 0
  pose.tripBreset()

  print("% Starting BOB mission! -------------------------")
  service.send("robobot/cmd/T0","leds 16 0 100 0") # green

  # Main "State Machine of the Mission"
  while not (service.stop):
      if state == 0: # wait for start signal
          service.send("robobot/cmd/ti","rc 0.2 0.0") # (forward m/s, turn-rate rad/sec)
          state = 1
      elif state == 1:
          edge.CUSTOM_CONTROL_ENABLED = True
          edge.lineControl(0.3)
      else:
          print(f"# drive 1m drove {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds")
          service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
          break
      print(f"# drive {state}, now {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds; left {edge.posLeft}, right {edge.posRight}")
      t.sleep(0.05)
  pass
  service.send("robobot/cmd/T0","leds 16 0 0 0") # end
  print("% Testing xy ------------------------- end")


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

def cameratest():
    #print("Starting camera test stream...")
    # 1. Connect to the robot's main camera stream
    print("hello cameratest ----------------------------------")
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
          #c v.putText(frame, "Custom Image Stream", (50, 300), cv.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 255), 3)
          gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)  
          _, thresh = cv.threshold(gray, 190, 255, cv.THRESH_BINARY)
          thresh_bgr = cv.cvtColor(thresh, cv.COLOR_GRAY2BGR)

          # 2. Encode to JPEG (Note: OpenCV uses BGR colorspace by default!)
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

    # Start the mini web server on port 7124
    address = ('0.0.0.0', 7125)
    print("Starting OpenCV test stream on port 7124...")
    httpd = ThreadedHTTPServer(address, TestStreamHandler)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down test stream.")
        httpd.server_close()


def custom_mission():

    # Resetting Odometry
    state = 0
    pose.tripBreset()

    print("% Starting our custom mission! -------------------------")
    service.send("robobot/cmd/T0","leds 16 0 100 0") # green

    # Main "State Machine of the Mission"
    while not (service.stop):
        if state == 0: # wait for start signal
            service.send("robobot/cmd/ti","rc 0.2 0.0") # (forward m/s, turn-rate rad/sec)
            state = 1
        elif state == 1:
            if pose.tripB > 1.0 or pose.tripBtimePassed() > 15:
                service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
                service.send("robobot/cmd/T0","servo 1 0 0") # (servo front fast)
                state = 2
            pass
        elif state == 2:
            if abs(pose.velocity()) < 0.001:
                state = 99
        else:
            print(f"# drive 1m drove {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds")
            service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
            break
        print(f"# drive {state}, now {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds; left {edge.posLeft}, right {edge.posRight}")
        t.sleep(0.05)
    pass
    service.send("robobot/cmd/T0","leds 16 0 0 0") # end
    print("% Custom mission ------------------------- end")


def LineTest():
  sky.state = 0
  pose.tripBreset()
  dist_to_line = 0
  print(f"% Ruta---------------------------------------------LineTest() start, LlineValidCnt, {edge.lineValidCnt:.1f}")
  print("% Driving to line ---------------------- right ir start ---")
  service.send("robobot/cmd/T0", "leds 16 0 100 0") # green
  while not (service.stop):
    if sky.state == 0: # forward towards line
      if ir.ir[0] > 0.2:
        service.send("robobot/cmd/ti","rc 0.1 0.0") # (forward m/s, turn-rate rad/sec)
        service.send("robobot/cmd/T0/","lognow 3") # (start Teensy log)
        service.send("robobot/cmd/T0","servo 1 -800 300") # (servo up slow)
        sky.state = 1
    elif sky.state == 1:
      if pose.tripB > 0.30 or pose.tripBtimePassed() > 15:
        service.send("robobot/cmd/ti/","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
        sky.state = 2
      if edge.lineValidCnt > 4:
        # start follow line
        edge.lineControl(0.3, True) # def lineControl(self, velocity, followLeft = True, refPosition = 0):
        service.send("robobot/cmd/T0","servo 1 0 0") # (move servo to position 0 - front)
        dist_to_line = pose.tripB
        pose.tripBreset()
        print(" to state 10")
        sky.state = 10
        sky.printcounter1 = 0 #RR
        sky.printcounter2 = 0 #RR
        sky.printcounter3 = 0 #RR
        sky.printcounter4 = 0 #RR
      pass
    elif sky.state == 2:
      if abs(pose.velocity()) < 0.001:
        print(" to state 99")
        sky.state = 99
    elif sky.state == 10:
      #print(f"% Ruta---------------------------------------------LineTest() STATE =10, ")
      if edge.lineValidCnt < 2:
        print(f"% Ruta---------------------------------------------LineTest() STATE =10, lineValidCnt < 2, {edge.lineValidCnt:.3f}")
        edge.lineControl(0, True)
        service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
        print(" to state 2")
        pose.tripBreset()
        sky.state = 2
    else:
      print(f"# drive to line {dist_to_line:.3f}m, then along line {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds")
      service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
      service.send("robobot/cmd/T0","servo 1 500 200") # (move servo down slow)
      break
    # print(f"# drive {state}, now {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds, line valid cnt = {edge.lineValidCnt}")
    t.sleep(0.01)
  pass
  service.send("robobot/cmd/T0","leds 16 0 0 0") # end
  print("% Driving to line ------------------------- end")




# This is the same function as in mqtt-client.py but as mqtt-client is not a valid name, the function could not be imported
def imageAnalysis(save):
  useCam = True
  if cam.useCam:
    ok, img, imgTime = cam.getImage()
    if not ok: # size(img) == 0):
      if cam.imageFailCnt < 5:
        print("% Failed to get image.")
    else:
      h, w, ch = img.shape
      if not service.args.silent:
        # print(f"% At {imgTime}, got image {cam.cnt} of size= {w}x{h}")
        pass
      edge.paint(img)
      if not gpio.onPi:
        try:
          cv.imshow('frame for analysis', img)
        except:
          print("% mqtt-client::imageAnalysis: failed to show camera image");
      if save:
        fn = f"image_{imgTime.strftime('%Y_%b_%d_%H%M%S_')}{cam.cnt:03d}.jpg"
        cv.imwrite(fn, img)
        if not service.args.silent:
          print(f"% Saved image {fn}")
      else:
        print("# imageAnalysis:: image not saved")
      pass
    pass
  service.stop = True
  pass



def my_signal_handler(sig, frame):
    print('UService:: You pressed Ctrl+C!')
    service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
    service.stop = True

def my_TimePassed(start):
  return (datetime.now() - start).total_seconds()



def randomtest():
  stopthis = False
  mystartTime = int(t.time())
  duration=0
  maxduration=5
  localcount = 0
  

  service.send("robobot/cmd/ti","rc 0.05 0.0") # (forward m/s, turn-rate rad/sec)
  while(not stopthis):
    signal.signal(signal.SIGINT, my_signal_handler)
    duration = int(t.time()) - mystartTime
    #print(f"% Ruta------------duration {duration:.1f} ")
    edge.LineDetect()
    if(datetime.now().microsecond % 100000==0):
      print(f"% Ruta------------LineDetect(),  LlineValidCnt, {edge.lineValidCnt:.1f} , lineValid  {edge.lineValid:.1f}")
      localcount = 0
    if(edge.lineValid>0 and localcount==0):
      print(f"% Ruta------------LineDetect(),  LlineValidCnt, {edge.lineValidCnt:.1f} , lineValid  {edge.lineValid:.1f}")
      localcount +=1
    if duration > maxduration:
      service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
      stopthis = True 
  service.stop = True



def controller():
  service.subscribe("robobot/controller")
  #while not (service.stop):
    
    #service.send("robobot/cmd/ti","rc 0.25 0.0")

  #  t.sleep(0.05)


def driveXY():
  pose.tripBreset()
  service.send("robobot/cmd/ti","rc 0.05 0.0")
  while not (service.stop):
    signal.signal(signal.SIGINT, my_signal_handler)
    if(datetime.now().microsecond % 100000==0):
            print(f"# pose.tripB0[1] {pose.tripB0}, pose.tripB1 {pose.tripB1}, pose.tripBh {pose.tripBh} rad ")
    #if pose.tripBh > (0.314):
    #  service.stop = True

    
   


"""
def driveXY():
  x=0.2
  y=0.3
  angle  = 0
  state = 15
  turnR = 0.4
  goalX = abs(x) - turnR
  goalY = abs(y) - turnR


  if goalY > 0:
    if goalX > 0:
      angle  = -np.arctan(x/y)
    elif goalX < 0:
      angle  = -np.arctan(x/y) - 3.14/2
    elif goalX == 0:

  elif goalX < 0:
    if goalY > 0:
      angle  = -np.arctan(x/y)
    elif goalY < 0:
      angle  = -np.arctan(x/y) - 3.14/2

  if goalX > 0 and goalY > 0:
    state=11
  elif goalX < 0:
    state=22
  
  if 
  

  
  pose.tripBreset()
  print("% Driving a Pi turn -------------------------")
  service.send("robobot/cmd/T0","leds 16 0 100 0") # green
  while not (service.stop):
    signal.signal(signal.SIGINT, my_signal_handler)
    if state == 15: #RR goal in  right upper quadrant
      service.send("robobot/cmd/ti","rc 0.05 0.0") # (forward m/s, turn-rate rad/sec)
      if pose.tripB > goalX:
        service.send("robobot/cmd/ti","rc 0.05 -0.1") # (forward m/s, turn-rate rad/sec)
        state = 12
    if state == 12:
      if(datetime.now().microsecond % 100000==0):
          print(f"# pose.tripBh {pose.tripBh} rad ")
      if pose.tripBh > np.arctan(x/y): #3.14/2: # has turned 90 degrees
          pose.tripBreset()
          service.send("robobot/cmd/ti","rc 0.05 0.0") # (forward m/s, turn-rate rad/sec)
          print(f"# to state 13")
          state = 13
    if state == 13:
      if(datetime.now().microsecond % 100000==0):
          print(f"# pose.tripB {pose.tripB} m ")
      if pose.tripB > goalY:
          service.stop = True
          service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
          print(f"# goal reached ")
    '''
    if state == 0: # wait for start signal
      service.send("robobot/cmd/ti","rc 0.2 0.5") # (forward m/s, turn-rate rad/sec)
      state = 1
    elif state == 1:
      if pose.tripBh > 3.14 or pose.tripBtimePassed() > 15:
        service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
        state = 2
      pass
    elif state == 2:
      if abs(pose.velocity()) < 0.001 and abs(pose.turnrate()) < 0.001:
        state = 99
    else:
      print(f"# drive turned {pose.tripBh:.3f} rad in {pose.tripBtimePassed():.3f} seconds")
      service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
      break
    print(f"# turn {state}, now {pose.tripBh:.3f} rad in {pose.tripBtimePassed():.3f} seconds; left {edge.posLeft}, right {edge.posRight}")
    t.sleep(0.05)
    '''
  pass
  service.send("robobot/cmd/T0","leds 16 0 0 0") #end
  print("% Driving a Pi turn ------------------------- end")
  service.stop = True
  """


