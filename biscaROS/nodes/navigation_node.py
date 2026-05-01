"""
The navigation node is responsible for estimating the robot's position and orientation on the track, using the Kalman filter from Henrik.
It subscribes to the MQTT topics that give measurements : gyro, wheel odometry and vision (for now only ArUco).
It publishes its estimates to another MQTT topic.
"""

# biscaROS/nodes/navigation.py
import time
import json
import numpy as np
from biscaROS.core.base_node import BaseNode

ENABLE_LOGGING = False

class NavigationNode(BaseNode):
    def __init__(self):
        super().__init__(node_name="navigation_node")
        
        # --- State Buffers ---
        self.acc = [0.0, 0.0, 0.0]
        self.wheelVelocity = [0.0, 0.0]
        self.wheelBase = 0.23 # Default from original config
        self.last_gyro_time = 0.0
        self.pos_meas = None
        self.att_meas = None
        
        # --- Kalman Filter Initialization ---
        self.X = np.array([0, 0, 0, 1.0, 0, 0, 0], dtype=np.float64) # x y z q_wxyz
        self.P = 0.0001 * np.eye(7) 
        self.acc_var = ((2.409e-01)/6)**2
        self.gyro_var = 1 * ((np.pi / 180) * 0.07) 
        self.vb_var = 0.01
        self.Sigma_u = np.diag([self.vb_var, self.gyro_var, self.gyro_var, self.gyro_var])
        self.R = np.eye(10)
        self.R[0:3, 0:3] *= 0.00000001
        self.R[3:6, 3:6] *= self.acc_var
        self.R[6:10, 6:10] *= 0.00000001

        self.fused_roll = 0.0
        self.fused_pitch = 0.0
        self.fused_yaw = 0.0
        self.fused_x = 0.0
        self.fused_y = 0.0
        self.fused_z = 0.0
        
        self.tripBh = 0.0
        self.tripAh = 0.0
        self.tripB_startX = 0.0
        self.tripB_startY = 0.0

        self.log_file_name = "robot_data.csv"
        self.log_initialized = False

        # --- Subscriptions ---
        self.client.subscribe("robobot/drive/T0/vel")
        self.client.subscribe("robobot/drive/T0/acc")
        self.client.subscribe("robobot/drive/T0/gyro")
        self.client.subscribe("biscaROS/vision/aruco/data")

    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode('utf-8')

        if topic == "robobot/drive/T0/vel":
            gg = payload.split(" ")
            if len(gg) > 3:
                self.wheelVelocity[0] = float(gg[2])
                self.wheelVelocity[1] = float(gg[3])

        elif topic == "robobot/drive/T0/acc":
            gg = payload.split(" ")
            if len(gg) >= 4:
                self.acc = [float(gg[1]), float(gg[2]), float(gg[3])]

        # elif topic == "biscaROS/vision/aruco/data":
        #     try:
        #         data = json.loads(payload)
        #         estimates = data.get("estimates", [])
        #         if len(estimates) > 0:
        #             # Use the first available marker estimate for correction
        #             est = estimates[0]
        #             self.pos_meas = [est["robot_x"], est["robot_y"], est["robot_z"]]
        #             self.att_meas = {'yaw': est["robot_yaw"]}
        #     except json.JSONDecodeError:
        #         pass

        elif topic == "robobot/drive/T0/gyro":
            gg = payload.split(" ")
            if len(gg) >= 4:
                t1 = float(gg[0])
                gyro = [float(gg[1]), float(gg[2]), float(gg[3])]

                if self.last_gyro_time > 0:
                    dt = t1 - self.last_gyro_time
                    if 0 < dt < 0.5:
                        self.fuse(self.acc, gyro, dt, pos_meas=self.pos_meas, att_meas=self.att_meas)
                        
                        # Consume vision measurements so they are not extrapolated infinitely
                        self.pos_meas = None
                        self.att_meas = None

                self.last_gyro_time = t1
        
        elif topic == "biscaROS/navigation/correct":
            try:
                data = json.loads(payload)
                self.reset_kalman(x=data.get("x", 0.0), y=data.get("y", 0.0), yaw=data.get("yaw", 0.0))
            except json.JSONDecodeError:
                pass

    # --- Kalman Filter Mathematics ---
    
    def predict(self, vb, omega, dt):
        qw, qx, qy, qz = self.X[3:]
        
        self.X[0] += (1 - 2*(qy**2 + qz**2)) * vb * dt
        self.X[1] += 2*(qx*qy + qw*qz) * vb * dt
        self.X[2] += 2*(qx*qz - qw*qy) * vb * dt

        ox, oy, oz = omega
        dq = 0.5 * np.array([
            -ox*qx - oy*qy - oz*qz,
            ox*qw + oz*qy - oy*qz,
            oy*qw - oz*qx + ox*qz,
            oz*qw + oy*qx - ox*qy
        ]) * dt
        
        self.X[3:] += dq
        self.X[3:] /= np.linalg.norm(self.X[3:])

        F = self.get_F(vb, omega, dt)
        W = self.get_W(dt)
        Q = W @ self.Sigma_u @ W.T
        self.P = F @ self.P @ F.T + Q

    def correct(self, acc_meas, pos_meas=None, att_meas=None):
        for i in range(3):
            if pos_meas is not None and pos_meas[i] is not None:
                self.R[i, i] = 0.001 
            else:
                self.R[i, i] = 10 
                pos_meas[i] = self.X[i] 
        
        for i in range(4):
            if att_meas is not None and att_meas[i] is not None:
                self.R[6+i, 6+i] = 0.001 
            else:
                self.R[6+i, 6+i] = 10 
                att_meas[i] = self.X[3+i] 

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

    def velocity(self):
        return (self.wheelVelocity[0] + self.wheelVelocity[1]) / 2

    def turnrate(self):
        return (self.wheelVelocity[0] - self.wheelVelocity[1]) / self.wheelBase

    def fuse(self, acc, gyro, dt, pos_meas=None, att_meas=None):
        if isinstance(pos_meas, dict):
            pos_meas = np.array([pos_meas.get('x'), pos_meas.get('y'), pos_meas.get('z')]) / 1000
        att_meas = self._normalize_attitude_measurement(att_meas)

        if not self.log_initialized and ENABLE_LOGGING:
            with open(self.log_file_name, 'w') as f:
                f.write("dt,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,fused_x,fused_y,fused_z,roll,pitch,yaw\n")
            self.log_initialized = True

        vb = self.velocity()
        gyro = np.array(gyro, dtype='float64') - np.array([-0.1834, -0.1299, -0.1700])
        omega = np.deg2rad(gyro) 

        acc_meas = np.array(acc, dtype='float64')
        acc_meas = np.array([
            acc_meas[0] - (-0.0149), 
            acc_meas[1] - (-0.0047), 
            acc_meas[2] / 1.2816     
        ])

        norm = np.linalg.norm(acc_meas)
        if norm > 1e-6:
            acc_meas /= norm
        
        pos_meas = [None, None, None] if pos_meas is None else pos_meas
        att_meas = [None, None, None, None] if att_meas is None else att_meas

        self.predict(vb, omega, dt)
        if norm > 1e-3: 
            self.correct(acc_meas, pos_meas=pos_meas, att_meas=att_meas)

        self.fused_x = self.X[0] 
        self.fused_y = self.X[1] 
        self.fused_z = self.X[2] 
        
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
        
        if ENABLE_LOGGING:
            try:
                data = [dt, *acc, *gyro, self.fused_x, self.fused_y, self.fused_z, 
                        self.fused_roll, self.fused_pitch, self.fused_yaw]
                log_line = ",".join(map(str, data))
                with open(self.log_file_name, 'a') as f:
                    f.write(log_line + "\n")
            except Exception:
                pass

        # Publish state estimations
        self.publish("biscaROS/navigation/pose", f"{self.fused_x} {self.fused_y} {self.fused_z}")
        self.publish("biscaROS/navigation/angle", f"{self.fused_roll} {self.fused_pitch} {self.fused_yaw}")
        
        # Maintain legacy topic output if required by other node implementations
        self.publish("robobot/map", f"{self.fused_x} {self.fused_y} {self.fused_yaw}")

    def reset_kalman(self, x, y, yaw):
        import numpy as np
        print(f"[Navigation] Hard reset to X:{x}, Y:{y}, Yaw:{yaw}")
        
        # Convert yaw to quaternion (assuming pitch=0, roll=0)
        q = self.euler_to_quaternion(0.0, 0.0, yaw)
        
        # Reset state vector: x, y, z, qw, qx, qy, qz
        self.X = np.array([x, y, 0.0, q[0], q[1], q[2], q[3]], dtype=np.float64)
        
        # Reset covariance (blow up uncertainty slightly to let filter settle, per your snippet)
        self.P = 10 * np.eye(7) 
        
        # Reset internal trackers
        self.fused_x = x
        self.fused_y = y
        self.fused_z = 0.0
        self.fused_roll = 0.0
        self.fused_pitch = 0.0
        self.fused_yaw = np.degrees(yaw)
        
        # Reset distance trackers so drive tasks don't calculate jumps
        self.tripB_startX = x
        self.tripB_startY = y
        self.tripBh = 0.0

if __name__ == "__main__":
    node = NavigationNode()
    node.start()
    
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        node.stop()