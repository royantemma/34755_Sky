#!/usr/bin/env python3

#/***************************************************************************
#*   Copyright (C) 2024 by DTU
#*   jcan@dtu.dk
#*
#*
#* The MIT License (MIT)  https://mit-license.org/
#*
#* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
#* and associated documentation files (the “Software”), to deal in the Software without restriction,
#* including without limitation the rights to use, copy, modify, merge, publish, distribute,
#* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
#* is furnished to do so, subject to the following conditions:
#*
#* The above copyright notice and this permission notice shall be included in all copies
#* or substantial portions of the Software.
#*
#* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
#* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
#* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#* THE SOFTWARE. */

#import sys
#import threading
import time as t
#import select
import numpy as np
import cv2 as cv
from datetime import *
from setproctitle import setproctitle
# robot function
import line_vision
from spose import pose
from sir import ir
from srobot import robot
from scam import cam
from sedge import edge
from sgpio import gpio
from scam import cam
from uservice import service

# Custom SKY114 Modules
import SKY114
import golf
import cube_catch
import calibrate_aruco
import find_aruco
import calibrate_extrinsics
import manual_controller
import calibrate_odom

############################################################

if __name__ == "__main__":
    if service.process_running("mqtt-client"):
      print("% mqtt-client is already running - terminating")
      print("%   if it is partially crashed in the background, then try:")
      print("%     pkill mqtt-client")
      print("%   or, if that fails use the most brutal kill")
      print("%     pkill -9 mqtt-client")
    else:
      # set title of process, so that it is not just called Python
      setproctitle("mqtt-client")
      print("% Starting")

      # where is the MQTT data server:
      service.setup('localhost') # localhost
      #service.setup('10.197.217.81') # Juniper
      #service.setup('10.197.217.80') # Newton
      # service.setup('bode.local') # Bode
      if service.connected:
        if service.args.mission_half:
          from missions.mission_half import TASKS, TOTAL_TIME, GOAL_TIME_BUFFER
          from mission_runner import MissionRunner
          MissionRunner(TASKS, TOTAL_TIME, GOAL_TIME_BUFFER).run()
          golf.find_and_catch()
        elif service.args.mission_half_HALF:
          from missions.mission_half_HALF import TASKS, TOTAL_TIME, GOAL_TIME_BUFFER
          from mission_runner import MissionRunner
          MissionRunner(TASKS, TOTAL_TIME, GOAL_TIME_BUFFER).run()
          golf.find_and_catch()
        elif service.args.mission_gates:
          from missions.mission_gates import TASKS, TOTAL_TIME, GOAL_TIME_BUFFER
          from mission_runner import MissionRunner
          MissionRunner(TASKS, TOTAL_TIME, GOAL_TIME_BUFFER).run()
        else:
          loop()
      service.terminate()
    print("% Main Terminated")



#/***************************************************************************
#*   Copyright (C) 2024 by DTU
#*   jcan@dtu.dk
#*
#*
#* The MIT License (MIT)  https://mit-license.org/
#*
#* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
#* and associated documentation files (the “Software”), to deal in the Software without restriction,
#* including without limitation the rights to use, copy, modify, merge, publish, distribute,
#* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
#* is furnished to do so, subject to the following conditions:
#*
#* The above copyright notice and this permission notice shall be included in all copies
#* or substantial portions of the Software.
#*
#* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
#* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
#* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#* THE SOFTWARE. */

## function to handle ctrl-C and reasonable shutdown
def signal_handler(sig, frame):
    print('UService:: You pressed Ctrl+C!')
    service.stop = True

import signal
import argparse
import time as t
import random
from paho.mqtt import client as mqtt_client
#from paho.mqtt.enums import CallbackAPIVersion
from datetime import *
from threading import Thread
#
from simu import imu
from spose import pose
from sir import ir
from srobot import robot
from scam import cam
from sedge import edge
from sgpio import gpio
from ulog import flog
import psutil

from sfuse import iwo

import globals

class UService:
  host = 'IP-setup'
  port = 1883
  topic = "robobot/drive/"
  topicCmd = "robobot/cmd/" # send to Teensy T0 (Teensy), T1, or ti (teensy_interface)
  topicCmdT0 = "robobot/cmd/T0/" # send to Teensy T0
  # Generate a Client ID with the subscribe prefix.
  client = []
  client_id = 'mqtt-client-in'
  clientOut = []
  clientOut_id = 'mqtt-client-out'
  connected = False
  connectedOut = False
  startTime = datetime.now()
  stop = False
  th = {} # thread for incoming
  th2 = {} # thread for outgoing
  thAlive = {} # thread for sending alive messages
  sendCnt = 0
  gotCnt = 0
  gotOutCnt = 0 # should continue to be 0
  failCnt = 0
  terminating = False
  confirmedMaster = False
  confirmedNotMaster = False
  parser = argparse.ArgumentParser(description='Robobot app 2024')

  def setup(self, mqtt_host):
    #
    print(self.startTime.strftime("Started %Y-%m-%d %H:%M:%S.%f"))
    from ulog import flog
    flog.setup()
    self.host = mqtt_host

    signal.signal(signal.SIGINT, signal_handler)
    ok = self.connect_mqtt()
    if not ok:
      return

    self.wait4mqttConnection()

    # start listening to incomming
    self.th = Thread(target=service.run);
    self.th.start()
    self.th2 = Thread(target=service.runOut);
    self.th2.start()
    self.thAlive = Thread(target=service.runAlive);
    self.thAlive.start()
    # do the setup and check of data streams
    # enable interface logging (into teensy_interface/build/log_2025...)
    service.send("robobot/cmd/ti", "log 1")

    return True


  def run(self):
    # print("% MQTT service - thread running")
    self.client.subscribe(self.topic + "#")
    # self.client.subscribe("robobot/controller")
    self.client.subscribe("robobot/#")
    print(f"subscribed to robobot/controller")
    self.client.on_message = self.on_message
    # self.subscribe(self.client)
    while not self.stop:
      self.client.loop()
    print("% Service - thread stopped")

  def runOut(self):
    # print("% MQTT service - out thread running")
    self.clientOut.on_message = self.on_messageOut
    while not self.stop:
      self.clientOut.loop()
    print("% Service - thread stopped")

  def runAlive(self):
    loop = 0;
    while not self.stop:
      # tell interface that we are alive
      if loop % 10 == 0:
        service.send("robobot/cmd/ti","alive " + str(service.startTime))
        # print(f"% sent Alive {datetime.now()}")
      if gpio.test_stop_button():
        self.stop = True
        print(f"this is the stop button")
      if gpio.get_value(19) == 1:
        print(f"start button is pressed $$$$$$$$$$$$$$$$$$$$$$$$$$$$")
      t.sleep(0.05)
      loop += 1
    pass


  def on_connect(self, client, userdata, flags, rc, properties = []):
    if rc == 0:
      print(f"% Connected to MQTT Broker {self.host} on {self.port}")
      self.connected = True
      self.client2=client
      # client.subscribe("robobot/controller")

  def on_connectOut(self, client, userdata, flags, rc, properties = []):
    if rc == 0:
      print(f"% ConnectedOut to MQTT Broker {self.host} on {self.port}")
      self.connectedOut = True
      self.clientOut2=client


  def connect_mqtt(self):
    import platform
    if platform.system() == "Windows":
      self.client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION2)
      self.clientOut = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION2)
    else:
      try:
        self.client = mqtt_client.Client(self.client_id)
        self.clientOut = mqtt_client.Client(self.clientOut_id)
        print("# MQTT Client in and out created OK (VERSION1)")
      except:
        self.client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION2)
        self.clientOut = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION2)
        # print("# MQTT Client uses - VERSION2")
    if isinstance(self.client, list):
      # print("# MQTT Failed to create MQTT client")
      return False
    # client.username_pw_set(username, password)
    self.client.on_connect = self.on_connect
    self.clientOut.on_connect = self.on_connectOut
    try:
      self.client.connect(self.host, self.port)
    except:
      print(f"% Failed to connect to {self.host} on {self.port}")
      print("% Won't work without MQTT connection, terminate!")
      return False
    try:
      self.clientOut.connect(self.host, self.port)
    except:
      print(f"% Failed to connect out to {self.host} on {self.port}")
      print("% Won't work without both MQTT connections, terminate!")
      return False
    return True

  def wait4mqttConnection(self):
    cnt = 0
    while not self.connected and self.connectedOut and cnt < 10:
      t.sleep(0.1)
      cnt += 1
    # print(f"% waited {cnt} times 0.1 sec");

  def on_message(self, client, userdata, msg):
    if True: #try:
      got = msg.payload.decode()
      used = self.decode(msg.topic, got)
      self.gotCnt += 1
      # if not used:
      #   print("# Message unused, topic '" + msg.topic + "' payload:" + str(msg.payload))
      # print("# Message (data), topic '" + msg.topic + "' payload:" + str(msg.payload))
    # except:
      # print("% Message in exception - continues, topic '" + msg.topic + "' payload:" + str(msg.payload))

  def on_messageOut(self, client, userdata, msg):
    try:
      got = msg.payload.decode()
      self.gotOutCnt += 1
      # print("# Message out, topic '" + msg.topic + "' payload:" + str(msg.payload))
    except:
      print("% Message exception (illegal char?) - continues, topic '" + msg.topic + "' payload:" + str(msg.payload))
    print(f"% MQTT got message on the output channel {msg.topic}")

  def decode(self, topic, msg):
    used = True # state.decode(msg, msgTime)
    if topic.startswith(self.topic):
      subtopic = topic[len(self.topic):]
      # print(f"# uservice::decode '{subtopic}' msg: '{msg}'")
      # flog.writeRemark(f"{subtopic} {msg}")
      if imu.decode(subtopic, msg):
        pass
      elif pose.decode(subtopic, msg):
        iwo.decode(subtopic,msg) # Henrik added this
        pass
      elif ir.decode(subtopic, msg):
        pass
      elif robot.decode(subtopic, msg):
        pass
      elif edge.decode(subtopic, msg):
        pass
      elif gpio.decode(subtopic, msg):
        pass
      elif subtopic == "T0/info":
        # if not self.args.silent:
          # print(f"% Teensy info {msg}", end="")
        pass
      elif subtopic == "master":
        # skip timestamp to get real masters starttime
        realMasterTime = msg[msg.find(" ")+1:]
        if str(self.startTime) == realMasterTime:
          if not self.confirmedMaster:
            print(f"% I am now accepted as master of robot {robot.robotName}")
          self.confirmedMaster = True
        else:
          self.confirmedNotMaster = True
          print("% I am not robot master, quitting!")
        # print(f"% got master {msg} my ID is {str(self.startTime)}")
        pass
      else:
        used = False
    # print(topic) # Commented out to prevent excessive terminal prints
    if "robobot/controller" == topic:
      used = True
      print(f"% got controller command")
      if msg == "forward":
        # globals.manual_controller_input = "w"
        manual_controller_input = "w"
        print(f"manual controller: forward")
      elif msg == "backward":
        # globals.manual_controller_input = "s"
        manual_controller_input = "s"
        print(f"manual controller: backward")
      elif msg == "left":
        # globals.manual_controller_input = "a"
        manual_controller_input = "a"
        print(f"manual controller: left")
      elif msg == "right":
        # globals.manual_controller_input = "d"
        manual_controller_input = "d"
        print(f"manual controller: right")
    if not used:
      print("% Service:: message not used " + topic + " " + msg)
    return used

  def send(self, topic, param):
    # print(self.startTime.strftime("At %Y-%m-%d %H:%M:%S.%f"))
    # print(f"% {self.startTime.strftime("At %Y-%m-%d %H:%M:%S.%f")}: sending: '{topic}' with '{param}' len(param)={len(param)}, not master {self.confirmedNotMaster}, master {self.confirmedMaster}")
    if self.confirmedNotMaster:
      # self.terminate()
      self.stop = True
      return False
    if len(param) == 0:
      param = " "
    r = self.clientOut.publish(topic, param)
    flog.writeRemark(f"{topic} {param}")
    if r[0] == 0:
      self.sendCnt += 1
      if self.sendCnt > 100 and self.gotCnt < 2:
        print(f"Seems like there is no connection to Teensy (tx:{self.sendCnt}, got:{self.gotCnt}); is Teensy_interface running?")
        self.stop = True
      # print(f"% published {topic} with {param}")
      pass
    else:
      print(f"% failed to publish {topic} with {param}")
      self.failCnt += 1
      if self.failCnt > 10:
        print("% Lost contact to MQTT server - terminating")
        self.stop = True
    return r[0] == 0
    pass

  def process_running(self, process_name):
    for process in psutil.process_iter(['pid', 'name']):
      if process.info['name'] == process_name:
        return True
    return False

  def terminate(self):
    from ulog import flog
    if self.terminating:
      return
    if not self.connected:
      return
    print("% shutting down")
    if self.connected and not self.confirmedNotMaster:
      edge.lineControl(0, 0) # make sure line control is off
      try:
        t.sleep(0.02)
        service.send("robobot/cmd/ti","rc 0 0") # stop robot control loop
        t.sleep(0.02)
        service.send("robobot/cmd/T0","stop") # should not be needed
        # turn off LEDs
        service.send("robobot/cmd/T0","leds 14 0 0 0")
        t.sleep(0.01)
        service.send("robobot/cmd/T0","leds 15 0 0 0")
        service.send("robobot/cmd/T0","leds 16 0 0 0")
        t.sleep(0.01)
        # stop interface logging
        service.send("robobot/cmd/ti", "log 0")
      except:
        print("% Failed to send terminate commands to robot - lost mqtt server connection?")
        pass
    self.terminating = True
    self.stop = True
    try:
      self.th.join()
    except:
      print("% Service thread not running")
    try:
      self.th2.join()
    except:
      print("% Service thread 2 not running")
    try:
      self.thAlive.join()
    except:
      print("% Service thread Alive not running")
    
    ## these might be terminated from the stream server
    imu.terminate()
    robot.terminate()
    pose.terminate()
    ir.terminate()
    edge.terminate()
    cam.terminate()
    gpio.terminate()
    flog.terminate()
    self.startTime = datetime.now()
    print(self.startTime.strftime("Ended at %Y-%m-%d %H:%M:%S.%f"))

# create the service object
service = UService()

