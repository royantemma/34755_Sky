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


import time as t
from datetime import *
import numpy as np
import aruco



class SImu:
    # AI
    aruco_client = None

    def _get_aruco_client(self):
      if self.aruco_client is None:
        self.aruco_client = aruco.get_client()
      return self.aruco_client

    def _get_aruco_pose(self):
      try:
        client = self._get_aruco_client()
        data = client.get_data()
        if not data or not data.get('estimates'):
          return None
        estimates = data['estimates']
        xs = [e['robot_x'] for e in estimates]
        ys = [e['robot_y'] for e in estimates]
        yaws = [e['robot_yaw'] for e in estimates]
        if len(estimates) == 0:
          return None
        avg_x = sum(xs) / len(xs)
        avg_y = sum(ys) / len(ys)
        avg_yaw = np.arctan2(np.sum(np.sin(yaws)), np.sum(np.cos(yaws)))
        return {'x': avg_x, 'y': avg_y, 'z': 0.0, 'yaw': avg_yaw}
      except Exception:
        return None
    ##
    gyro = [0, 0, 0]
    gyroUpdCnt = 0
    gyroTime = datetime.now()
    gyroInterval = 1

    acc  = [0, 0, 0]
    accTime = datetime.now()
    accUpdCnt = 0
    accInterval = 1

    def setup(self):
      # data subscription is set in teensy_interface/build/robot.ini
      from uservice import service
      loops = 0
      while not service.stop:
        t.sleep(0.01)
        if self.gyroUpdCnt == 0 or self.accUpdCnt == 0:
          # wait for data
          pass
        else: # finished
          print(f"% IMU (simu.py):: got data stream; {loops} loops.")
          break
        loops += 1
        if loops > 20:
          print(f"% IMU (simu.py):: no data updates after {loops} wait loops (continues).")
          break
        pass
        loops += 1
      # should we calibrate the gyro
      if service.args.gyro:
        print("% Starting calibrate gyro offset.")
        # ask Teensy to calibrate
        service.send("robobot/cmd/T0", "gyroc")
        # wait for calibration to finish (average over 1s)
        t.sleep(2.5)
        # save calibrated values
        service.send("robobot/cmd/T0", "eew")
        print("% Starting calibrate gyro offset finished.")
        t.sleep(0.5)
        # all done
        service.stop = True
      pass

    def print(self):
      from uservice import service
      print("% IMU acc  " + str(self.accTime - service.startTime) + " (" +
            str(self.acc[0]) + ", " +
            str(self.acc[1]) + ", " +
            str(self.acc[2]) + f") {self.gyroInterval:.4f} sec " +
            str(self.accUpdCnt))
      print("% IMU gyro " + str(self.gyroTime - service.startTime) + " (" +
            str(self.gyro[0]) + ", " +
            str(self.gyro[1]) + ", " +
            str(self.gyro[2]) + f") {self.accInterval:.4f} sec " +
            str(self.gyroUpdCnt))

    def decode(self, topic, msg):
        # decode MQTT message
        used = True
        if topic == "T0/gyro":
          gg = msg.split(" ")
          if (len(gg) >= 4):
            t0 = self.gyroTime;
            self.gyroTime = datetime.fromtimestamp(float(gg[0]))
            self.gyro[0] = float(gg[1])
            self.gyro[1] = float(gg[2])
            self.gyro[2] = float(gg[3])
            t1 = self.gyroTime;
            if self.gyroUpdCnt == 2:
              self.gyroInterval = (t1 -t0).total_seconds()
            else:
              self.gyroInterval = (self.gyroInterval * 99 + (t1 -t0).total_seconds()) / 100
            self.gyroUpdCnt += 1
            # self.print()

            from sfuse import iwo
            if iwo.wheelVelocityCnt > 0:
              dt = (t1 - t0).total_seconds()
              if 0 < dt:# < 0.5: # Handle if t0 lies in the past
                  if dt > 0.5:
                    print(f"% Warning: large dt={dt:.3f} sec between gyro updates!")
                  aruco_pose = self._get_aruco_pose()
                  pos_meas = None
                  att_meas = None
                  #if aruco_pose is not None:
                  #  pos_meas = [aruco_pose['x']/1000, aruco_pose['y']/1000, aruco_pose['z']/1000]
                  #  #pos_meas = [aruco_pose['x'], aruco_pose['y'], aruco_pose['z']]
                  #  att_meas = [np.deg2rad(aruco_pose['yaw'])]
                  iwo.fuse(self.acc, self.gyro, dt, pos_meas=pos_meas, att_meas=att_meas)
                  #print(f"Fused with dt={dt:.3f} sec, gyro={self.gyro}, acc={self.acc}, aruco_pose={aruco_pose}, fused_x={iwo.fused_x:.3f}, fused_y={iwo.fused_y:.3f}, fused_yaw={iwo.fused_yaw:.2f}")


        elif topic == "T0/acc":
          gg = msg.split(" ")
          if (len(gg) >= 4):
            t0 = self.accTime;
            self.accTime = datetime.fromtimestamp(float(gg[0]))
            self.acc[0] = float(gg[1])
            self.acc[1] = float(gg[2])
            self.acc[2] = float(gg[3])
            t1 = self.accTime;
            if self.accUpdCnt == 2:
               self.accInterval = (t1 -t0).total_seconds()
            else:
               self.accInterval = (self.accInterval * 99 + (t1 -t0).total_seconds()) / 100
            self.accUpdCnt += 1
            # self.print()
        else:
          used = False
        return used

    def terminate(self):
        print("% Pose terminated")
        pass

# create the data object
imu = SImu()

