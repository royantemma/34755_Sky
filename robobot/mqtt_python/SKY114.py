import time as t
import numpy as np
import cv2 as cv
from datetime import *
from setproctitle import setproctitle
import signal

import sys
sys.path.append("/home/local/svn/robobot/stream_server")
from stream_server import stream_manager
import threading
import simplejpeg
from scam import getImage


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

####################################################################

def cameratest():
  cap = cv.VideoCapture("http://localhost:7123/stream/main")

  ret, frame = cap.read()
  print("$$$$$$$$$$$$$")
  print(ret)


  cameratest_output = stream_manager.get_output("cameratest")

  def push_processed_images():
    for i in range(100):
      # Generate or process any image you like
      #ok, img, frameTime = getImage()
      #print(str(i) + ', ' + str(ok))
      if ret and frame is not None:
      #if ok and img is not None:
        cv.putText(frame, "Custom Image Stream", (50, 300),
                    cv.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 255), 3)

        # Encode to JPEG
        jpeg = simplejpeg.encode_jpeg(frame, quality=80)
        cameratest_output.write(jpeg)

      t.sleep(0.2)  # ~5 FPS

  threading.Thread(target=push_processed_images, daemon=True).start()
  service.stop = True



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


