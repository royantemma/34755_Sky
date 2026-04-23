import time as t
from datetime import *
from threading import Thread
import numpy as np

ENABLE_LOGGING = False

class KalmanFilter:
    #
    motorVelocity = [0,0] # in radians/sec
    motorVelocityTime = datetime.now()
    motorVelocityCnt = 0
    motorVelocityInterval = 1000 # sec
    #
    wheelVelocity = [0,0] # in m/sec - if gearing and wheel radius is correct
    wheelVelocityTime = datetime.now()
    wheelVelocityCnt = 0
    wheelVelocityInterval = 1000 # sec
    #
    tripA = 0; # should not be reset
    tripAh = 0; # heading
    tripAtime = datetime.now()
    tripB_startX = 0.0
    tripB_startY = 0.0
    tripBh = 0; # heading
    tripBtime = datetime.now()
    # robot info
    infoTime = datetime.now()
    infoCnt = 0
    tickPerRev = 68
    radiusLeft = 0.1
    radiusRight = 0.1
    gear = 19
    wheelBase = 0.1
    encoder_reversed = False
    need_data = True
    # pose (x (m),y (m),h (rad), tilt (rad - if available))
    pose = [0.0, 0.0, 0.0, 0.0]
    poseTime = datetime.now()
    poseCnt = 0
    poseInterval = 1000 # sec

    # Initialize Kalman
    # Delta_t = 12 * 10e-3
    X = np.array([0, 0, 0, 1.0, 0, 0, 0]) # x y z q_wxyz
    P = 0.0001*np.eye(7) # Initial Error Covariance small because we know it starts at 0,0,0 with no rotation
    acc_var = ((2.409e-01)/6)**2
    gyro_var = 1*(( np.pi /180) * (0.07) ) 
    vb_var = 0.001
    Sigma_u = np.diag([vb_var, gyro_var,gyro_var,gyro_var]) # Process Noise Covariance
    R = np.eye(10) # Measurement Noise Covariance
    R[0:3, 0:3] *= 0.00000001 # Noise Pos
    R[3:6, 3:6] *= acc_var  # Noise in Acc
    R[6:10, 6:10] *= 0.00000001 # Noise Att

    fused_roll = 0
    fused_pitch = 0
    fused_yaw = 0
    fused_x = 0
    fused_y = 0
    fused_z = 0

    # Logging
    log_file_name = "robot_data.csv"
    log_initialized = False

    def predict(self, vb, omega, dt):
      
      qw, qx, qy, qz = self.X[3:]
      
      self.X[0] += (1 - 2*(qy**2 + qz**2)) * vb * dt
      self.X[1] += 2*(qx*qy + qw*qz) * vb * dt
      self.X[2] += 2*(qx*qz - qw*qy) * vb * dt

      # Quaternion-Update: q_new = q + 0.5 * Omega * q * dt
      ox, oy, oz = omega
      dq = 0.5 * np.array([
          -ox*qx - oy*qy - oz*qz,
          ox*qw + oz*qy - oy*qz,
          oy*qw - oz*qx + ox*qz,
          oz*qw + oy*qx - ox*qy
      ]) * dt
      self.X[3:] += dq
      self.X[3:] /= np.linalg.norm(self.X[3:]) # Normalize to get valid quaternion

      F = self.get_F(vb, omega, dt)
      W = self.get_W(dt)
      Q = W @ self.Sigma_u @ W.T

      self.P = F @ self.P @ F.T + Q

    def correct(self, acc_meas, pos_meas=None, att_meas=None):
     
      for i in range(3):
        if pos_meas is not None and pos_meas[i] is not None:
          self.R[i, i] = 0.0001 # Very low noise for position measurements
        else:
          self.R[i, i] = 10 # High noise for missing position measurements
          pos_meas[i] = self.X[i] # Use predicted position as measurement if not provided
      
      # Attitude measurements as quaternion
      for i in range(4):
        if att_meas is not None and att_meas[i] is not None:
          self.R[6+i, 6+i] = 0.0001 # Very low noise for attitude measurements
        else:
          self.R[6+i, 6+i] = 10 # High noise for missing attitude measurements
          att_meas[i] = self.X[3+i] # Use predicted attitude as measurement if not provided

      z_t = np.concatenate([pos_meas, acc_meas, att_meas])
      
      H = self.get_H()
      
      v_t = z_t - self.get_h()
      
      S_t = H @ self.P @ H.T + self.R
      K_t = self.P @ H.T @ np.linalg.inv(S_t)

      self.X = self.X + K_t @ v_t
      self.P = (np.eye(7) - K_t @ H) @ self.P
      
      self.X[3:] /= np.linalg.norm(self.X[3:])
    
    def get_F(self, vb, omega, dt):
    
      qw, qx, qy, qz = self.X[3:7]
      ox, oy, oz = omega  

      s1 = -0.5 * dt * oz
      s2 = -0.5 * dt * oy
      s3 = -0.5 * dt * ox
      
      # Matrix F
      F = np.array([
          [1, 0, 0, 0, 0, -4*dt*qy*vb, -4*dt*qz*vb],
          [0, 1, 0, 2*dt*qz*vb, 2*dt*qy*vb, 2*dt*qx*vb, 2*dt*qw*vb],
          [0, 0, 1, -2*dt*qy*vb, 2*dt*qz*vb, -2*dt*qw*vb, 2*dt*qx*vb],
          [0, 0, 0, 1, s3, s2, s1],
          [0, 0, 0, 0.5*dt*ox, 1, 0.5*dt*oz, s2],
          [0, 0, 0, 0.5*dt*oy, s1, 1, 0.5*dt*ox],
          [0, 0, 0, 0.5*dt*oz, 0.5*dt*oy, s3, 1]
      ])
      return F

    def get_W(self, dt):
      
      qw, qx, qy, qz = self.X[3:7]
      
      s1 = -0.5 * dt * qz
      s2 = -0.5 * dt * qy
      s3 = -0.5 * dt * qx
      
      # Matrix W       
      W = np.array([
          [-dt * (2*qy**2 + 2*qz**2 - 1), 0, 0, 0],
          [ dt * (2*qw*qz + 2*qx*qy),      0, 0, 0],
          [-dt * (2*qw*qy - 2*qx*qz),      0, 0, 0],
          [0, s3, s2, s1],
          [0, 0.5*dt*qw, s1, 0.5*dt*qy],
          [0, 0.5*dt*qz, 0.5*dt*qw, s3],
          [0, s2, 0.5*dt*qx, 0.5*dt*qw]
      ])
      return W

    def get_h(self, g=[0, 0, 1]):
      x, y, z, qw, qx, qy, qz = self.X
      gx, gy, gz = g
      
      ax_exp = 2*gy*(qw*qz + qx*qy) - 2*gz*(qw*qy - qx*qz) - 2*gx*(qy**2 + qz**2 - 0.5)
      ay_exp = 2*gz*(qw*qx + qy*qz) - 2*gx*(qw*qz - qx*qy) - 2*gy*(qx**2 + qz**2 - 0.5)
      az_exp = 2*gx*(qw*qy + qx*qz) - 2*gy*(qw*qx - qy*qz) - 2*gz*(qx**2 + qy**2 - 0.5)
      
      return np.array([x, y, z, ax_exp, ay_exp, az_exp, qw, qx, qy, qz])

    def get_H(self, g=[0, 0, 1]):
      qw, qx, qy, qz = self.X[3:7]
      gx, gy, gz = g
      
      #H = np.zeros((6, 7))
      #H = np.zeros((7, 7))
      H = np.zeros((10, 7))
      
      H[0:3, 0:3] = np.eye(3)
      
      H[3, 3] =  2*gy*qz - 2*gz*qy
      H[3, 4] =  2*gy*qy + 2*gz*qz
      H[3, 5] =  2*gy*qx - 4*gx*qy - 2*gz*qw
      H[3, 6] =  2*gy*qw - 4*gx*qz + 2*gz*qx
      
      H[4, 3] =  2*gz*qx - 2*gx*qz
      H[4, 4] =  2*gx*qy - 4*gy*qx + 2*gz*qw
      H[4, 5] =  2*gx*qx + 2*gz*qz
      H[4, 6] =  2*gz*qy - 4*gy*qz - 2*gx*qw
      
      H[5, 3] =  2*gx*qy - 2*gy*qx
      H[5, 4] =  2*gx*qz - 2*gy*qw - 4*gz*qx
      H[5, 5] =  2*gx*qw + 2*gy*qz - 4*gz*qy
      H[5, 6] =  2*gx*qx + 2*gy*qy
      
      """
      # Yaw
      c_1 = 2 * qy**2 + 2*qz**2 - 1
      c_2 = 2 * qw*qz + 2*qx*qy
      s = -2 * (c_1**2 + c_2**2)
      H[6, 3] = 2 * qz * c_1 / s 
      H[6, 4] = 2 * qy * c_1 / s 
      H[6, 5] = (2*qx*c_1 - 4*qy*c_2) / s 
      H[6, 6] =  (2*qw*c_1 - 4*qz*c_2) / s 
      """
      # All Orientation
      H[6:, 3:] = np.eye(4)

      
      return H

    def euler_to_quaternion(self, roll=0.0, pitch=0.0, yaw=0.0):
      cr = np.cos(roll * 0.5)
      sr = np.sin(roll * 0.5)
      cp = np.cos(pitch * 0.5)
      sp = np.sin(pitch * 0.5)
      cy = np.cos(yaw * 0.5)
      sy = np.sin(yaw * 0.5)
      qw = cr * cp * cy + sr * sp * sy
      qx = sr * cp * cy - cr * sp * sy
      qy = cr * sp * cy + sr * cp * sy
      qz = cr * cp * sy - sr * sp * cy
      return [qw, qx, qy, qz]

    def _normalize_attitude_measurement(self, att_meas):
      if att_meas is None:
        return None
      if isinstance(att_meas, dict):
        if 'yaw' in att_meas and 'pitch' not in att_meas and 'roll' not in att_meas:
          return self.euler_to_quaternion(0.0, 0.0, att_meas['yaw'])
        if all(key in att_meas for key in ('roll', 'pitch', 'yaw')):
          return self.euler_to_quaternion(att_meas['roll'], att_meas['pitch'], att_meas['yaw'])
        if all(key in att_meas for key in ('qw', 'qx', 'qy', 'qz')):
          return [att_meas['qw'], att_meas['qx'], att_meas['qy'], att_meas['qz']]
      if isinstance(att_meas, (list, tuple, np.ndarray)):
        if len(att_meas) == 1:
          return self.euler_to_quaternion(0.0, 0.0, att_meas[0])
        if len(att_meas) == 3:
          return self.euler_to_quaternion(att_meas[0], att_meas[1], att_meas[2])
        if len(att_meas) == 4:
          return list(att_meas)
      return att_meas

    def fuse(self, acc, gyro, dt, pos_meas=None, att_meas=None):
      if isinstance(pos_meas, dict):
        pos_meas = np.array([pos_meas.get('x'), pos_meas.get('y'), pos_meas.get('z')]) / 1000
      att_meas = self._normalize_attitude_measurement(att_meas)
      if pos_meas is not None and len(pos_meas) != 3:
        raise ValueError('pos_meas must be a 3-element list, tuple, or dict with x/y/z')
      if att_meas is not None and len(att_meas) != 4:
        raise ValueError('att_meas must be a 4-element quaternion or yaw-only list/dict')

      if not self.log_initialized:
        with open(self.log_file_name, 'w') as f:
            f.write("dt,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,fused_x,fused_y,fused_z,roll,pitch,yaw,pose_x,pose_y,pose_yaw\n")
        self.log_initialized = True

      #print(pos_meas)
      #print(att_meas)
      vb = self.velocity()
      gyro = np.array(gyro, dtype='float64') - np.array([-0.1834, -0.1299, -0.1700])
      # Convert to radians/sec and apply bias correction
      omega = np.deg2rad(gyro) 

      #print(f"vb: {vb} m/s, omega: {omega} rad/s, dt: {dt} sec")
      acc_meas = np.array(acc, dtype='float64')
      #print(acc_meas)
      
      # Normalize accelerometer reading & corrct bias
      acc_meas = np.array([
        acc_meas[0] - (-0.0149), # X-Bias 
        acc_meas[1] - (-0.0047), # Y-Bias 
        acc_meas[2] / 1.2816     # Z-Scale
      ])

      norm = np.linalg.norm(acc_meas)
      if norm > 1e-6:
          acc_meas /= norm

          #print(acc_meas)
      
      pos_meas = [None, None, None] if pos_meas is None else pos_meas
      att_meas = [None, None, None, None] if att_meas is None else att_meas

      self.predict(vb, omega, dt)
      if norm > 1e-3: 
        self.correct(acc_meas, pos_meas=pos_meas, att_meas=att_meas)
      else:
          print("Warning: Skipping correction due to zero acceleration")
      #self.correct(acc_meas, pos_meas=None)

      self.fused_x = self.X[0] # x
      self.fused_y = self.X[1] # y
      self.fused_z = self.X[2] # z
      
      qw, qx, qy, qz = self.X[3:]
      last_yaw = self.fused_yaw
      self.fused_yaw = np.rad2deg(np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2)))
      self.fused_pitch = np.rad2deg(np.arcsin(2*(qw*qy - qz*qx)))
      self.fused_roll = np.rad2deg(np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2)))
      
      dh = np.deg2rad(self.fused_yaw - last_yaw)
      if dh > np.pi:
        dh -= 2.0 * np.pi
      elif dh < -np.pi:
        dh += 2.0 * np.pi
      self.tripBh += dh
      self.tripAh += dh
      
      try:

        if ENABLE_LOGGING:
          data = [dt, *acc, *gyro, self.fused_x, self.fused_y, self.fused_z, 
                  self.fused_roll, self.fused_pitch, self.fused_yaw, self.pose[0], self.pose[1], self.pose[2]]
          
          log_line = ",".join(map(str, data))
          
          with open(self.log_file_name, 'a') as f:
              f.write(log_line + "\n")
      except Exception as e:
        print(f"Logging Error: {e}")

    def reset_kalman(self):
      self.X = np.array([0, 0, 0, 1.0, 0, 0, 0]) # x y z q_wxyz
      self.P = 10*np.eye(7) # Initial Error Covariance small because we know it starts at 0,0,0 with no rotation
      self.acc_var = ((2.409e-01)/6)**2
      self.gyro_var = 1*(( np.pi /180) * (0.07) ) 
      self.vb_var = 0.001
      self.Sigma_u = np.diag([self.vb_var, self.gyro_var,self.gyro_var,self.gyro_var]) # Process Noise Covariance
      self.R = np.eye(10) # Measurement Noise Covariance
      self.R[0:3, 0:3] *= 0.00000001 # Noise Pos
      self.R[3:6, 3:6] *= self.acc_var  # Noise in Acc
      self.R[6:10, 6:10] *= 0.00000001 # Noise Att

      self.fused_roll = 0
      self.fused_pitch = 0
      self.fused_yaw = 0
      self.fused_x = 0
      self.fused_y = 0
      self.fused_z = 0


    def velocity(self):
      # meters per second
      return (self.wheelVelocity[0] + self.wheelVelocity[1])/2

    def turnrate(self):
      # radians per second
      return (self.wheelVelocity[0] - self.wheelVelocity[1])/self.wheelBase

    def printMVel(self):
      print("% Pose motor velocity (" +
            str(self.motorVelocity[0]) + ", " +
            str(self.motorVelocity[1]) + f") (rad/sec) {self.motorVelocityInterval:.4f} sec " +
            str(self.motorVelocityCnt))

    def printWVel(self):
      print("% Pose wheel velocity (" +
            str(self.wheelVelocity[0]) + ", " +
            str(self.wheelVelocity[1]) + f") (m/sec) {self.wheelVelocityInterval:.4f} sec " +
            str(self.wheelVelocityCnt))

    def printPose(self):
      print("% Pose (" +
            f"{self.pose[0]:.3f}, " +
            f"{self.pose[1]:.3f}, " +
            f"{self.pose[2]:.4f}, " +
            f"{self.pose[3]:.4f}) (m,m,rad,rad) {self.poseInterval:.4f} sec " +
            str(self.poseCnt))

    def printInfo(self):
      print(f"% SPose.py:: Robot config info {self.infoCnt}")
      print(f"%    - Wheel radius (left,right): ({self.radiusLeft}, {self.radiusRight} m)")
      print(f"%    - Encoder tick per rev: {self.tickPerRev}")
      print(f"%    - Gearing: {self.gear}:1")
      print(f"%    - Wheel base: {self.wheelBase} m")
      # reversed is for motortest only
      # print(f"%    - Encoder reversed: {self.encoder_reversed} (1 = reversed)")

    def tripAreset(self):
      self.tripA = 0
      self.tripAh = 0
      self.tripAtime = datetime.now()

    @property
    def tripB(self):
      # True Euclidean displacement based on Kalman fusion snapshot
      return np.sqrt((self.fused_x - self.tripB_startX)**2 + (self.fused_y - self.tripB_startY)**2)

    def tripBreset(self):
      self.tripB_startX = self.fused_x
      self.tripB_startY = self.fused_y
      self.tripBh = 0
      self.tripBtime = datetime.now()

    def tripAtimePassed(self):
      return (datetime.now() - self.tripAtime).total_seconds()

    def tripBtimePassed(self):
      return (datetime.now() - self.tripBtime).total_seconds()

    def decode(self, topic, msg):
        from ulog import flog
        # decode MQTT message
        used = True
        if topic == "T0/vel":
          gg = msg.split(" ")
          if (len(gg) > 3):
            t0 = self.wheelVelocityTime;
            self.wheelVelocityTime = datetime.fromtimestamp(float(gg[0]))
            # Teensy time (gg[1]) is ignored
            self.wheelVelocity[0] = float(gg[2])
            self.wheelVelocity[1] = float(gg[3])
            dt = (self.wheelVelocityTime - t0).total_seconds();
            if self.wheelVelocityCnt == 2:
              self.wheelVelocityInterval = dt;
            else:
              self.wheelVelocityInterval = (self.wheelVelocityInterval * 99 + dt) / 100
            self.wheelVelocityCnt += 1
            ds = (self.wheelVelocity[0] + self.wheelVelocity[1])*dt/2
            self.tripA += ds
            # tripB accumulation removed: now dynamically computed from Kalman snapshot
            # self.printMVel()
        elif topic == "T0/mvel":
          gg = msg.split(" ")
          if (len(gg) > 2):
            t0 = self.motorVelocityTime
            self.motorVelocityTime = datetime.fromtimestamp(float(gg[0]))
            self.motorVelocity[0] = float(gg[1])
            self.motorVelocity[1] = float(gg[2])
            dt = (self.wheelVelocityTime - t0).total_seconds();
            if self.motorVelocityCnt == 2:
              self.motorVelocityInterval = dt
            else:
              self.motorVelocityInterval = (self.motorVelocityInterval * 99 + dt) / 100
            self.motorVelocityCnt += 1
            # self.printWVel()
        elif topic == "T0/pose":
          gg = msg.split(" ")
          if (len(gg) > 5):
            t0 = self.poseTime
            self.poseTime = datetime.fromtimestamp(float(gg[0]))
            # parameter 1 is teensy timestamp, ignored
            self.pose[0] = float(gg[2])
            self.pose[1] = float(gg[3])
            # heading
            h = float(gg[4])
            dh = h - self.pose[2]
            if (dh > np.pi):
              dh -= 2.0 * np.pi
            elif (dh < -np.pi):
              dh += 2.0 * np.pi
            self.pose[2] = h
            # tilt
            self.pose[3] = float(gg[5])
            # update statistics
            dt = (self.poseTime - t0).total_seconds();
            if self.poseCnt == 2:
              self.poseInterval = dt
            else:
              self.poseInterval = (self.poseInterval * 99 + dt) / 100
            self.poseCnt += 1
            # save pose to log
            if self.poseCnt % 10 == 0:
              flog.write()
            # self.printPose()
        elif topic == "T0/conf":
          # from Teensy: snprintf(s, MSL, "conf %.4f %.4f %.3f %u %.4f %.4f %d\r\n", odoWheelRadius[0], odoWheelRadius[1],
          #   gear, pulsPerRev, odoWheelBase, float(service.sampleTime_us)/1e6, motor.motorReversed
          gg = msg.split(" ")
          if (len(gg) > 7):
            self.infoTime = datetime.fromtimestamp(float(gg[0]))
            self.radiusLeft = float(gg[1])
            self.radiusRight = float(gg[2])
            self.gear = float(gg[3])
            self.tickPerRev = float(gg[4])
            self.wheelBase = float(gg[5])
            self.encoder_reversed = float(gg[7])
            self.infoCnt += 1
            # if self.infoCnt == 1:
            self.printInfo()
          else:
            print("% SPose:: got a too short configuration message '{msg}'")
        else:
          used = False
        return used

    def terminate(self):
        print("% Pose terminated")
        pass

# create the data object
iwo = KalmanFilter()

