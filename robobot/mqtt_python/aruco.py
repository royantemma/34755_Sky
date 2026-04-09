import numpy as np
import cv2 as cv
import os
import urllib.request
import json
import time

class ArucoProcessor:
    """Handles all intrinsic and extrinsic camera calibration, and finding ArUco cubes."""
    
    def __init__(self):
        self.mtx, self.dist = self._load_calibration()
        self.R_cam2rob, self.T_cam2rob = self._get_extrinsic_matrix()
        
        self.marker_size = 0.035
        self.obj_points = np.array([
            [-self.marker_size/2,  self.marker_size/2, 0],
            [ self.marker_size/2,  self.marker_size/2, 0],
            [ self.marker_size/2, -self.marker_size/2, 0],
            [-self.marker_size/2, -self.marker_size/2, 0]
        ], dtype=np.float32)

        self.aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
        if hasattr(cv.aruco, 'DetectorParameters_create'):
            self.aruco_params = cv.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv.aruco.DetectorParameters()

    def _load_calibration(self):
        calib_dir = os.path.dirname(os.path.abspath(__file__))
        calib_file = os.path.join(calib_dir, "calibration_checkerboard.npz")
        fallback_file = os.path.join(calib_dir, "calibration_data.npz")
        
        mtx, dist = None, None
        if os.path.exists(calib_file):
            with np.load(calib_file) as X:
                mtx, dist = [X[i] for i in ('mtx', 'dist')]
        elif os.path.exists(fallback_file):
            with np.load(fallback_file) as X:
                mtx, dist = [X[i] for i in ('mtx', 'dist')]
        return mtx, dist

    def _get_extrinsic_matrix(self):
        # Pre-compute Camera to Robot transformation matrix
        theta = np.radians(20.09) # Pitch (around x-axis)
        phi = np.radians(1.4)     # Tilt around y-axis (towards the right)
        cam_z_offset = 236.9
        cam_y_offset = 20.0       # 20 mm in front of origin
        
        # Camera tilt/yaw (rotation around Y)
        R_yaw = np.array([
            [np.cos(phi), 0, np.sin(phi)],
            [0, 1, 0],
            [-np.sin(phi), 0, np.cos(phi)]
        ])
        
        # Base camera to robot conversion: X_rob = X_cam, Y_rob = Z_cam, Z_rob = -Y_cam
        # With pitch (theta) applied:
        R_pitch_base = np.array([
            [1, 0, 0],
            [0, -np.sin(theta), np.cos(theta)],
            [0, -np.cos(theta), -np.sin(theta)]
        ])
        
        R_cam2rob = R_pitch_base @ R_yaw
        T_cam2rob = np.array([[0.0], [cam_y_offset], [cam_z_offset]])
        return R_cam2rob, T_cam2rob

    def process_image(self, frame, valid_ids=None):
        """
        Detects ArUco markers in the provided frame, returns an annotated frame,
        and the structured JSON-compatible data with coordinates.
        """
        if self.mtx is None or self.dist is None:
            return frame, {"markers": [], "cube_center": None}
            
        if valid_ids is None:
            valid_ids = {5, 20, 53} | set(range(10, 18))

        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        try:
            detector = cv.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            corners, ids, _ = detector.detectMarkers(gray)
        except AttributeError:
            corners, ids, _ = cv.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        current_data = []
        cube_centers_data = []

        if ids is not None:
            cv.aruco.drawDetectedMarkers(frame, corners, ids)
            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                if marker_id not in valid_ids:
                    continue
                
                marker_corners = corners[i][0]
                ret, rvec, tvec = cv.solvePnP(self.obj_points, marker_corners, self.mtx, self.dist)
                if ret:
                    cv.drawFrameAxes(frame, self.mtx, self.dist, rvec, tvec, self.marker_size)
                    
                    # 1. Get Marker Center in Camera Frame (in mm)
                    tvec_mm = tvec * 1000.0
                    
                    # 2. Get Cube Center in Camera Frame (in mm)
                    # 30mm behind the marker along the marker's Z-axis
                    R_marker, _ = cv.Rodrigues(rvec)
                    offset_marker_frame = np.array([[0.0], [0.0], [-30.0]])
                    cube_center_cam_mm = tvec_mm + (R_marker @ offset_marker_frame)
                    
                    # 3. Transform to Robot Frame
                    marker_rob = (self.R_cam2rob @ tvec_mm) + self.T_cam2rob
                    cube_rob = (self.R_cam2rob @ cube_center_cam_mm) + self.T_cam2rob
                    
                    x_rob, y_rob, z_rob = marker_rob.flatten()
                    cx_rob, cy_rob, cz_rob = cube_rob.flatten()
                    
                    current_data.append({
                        "id": marker_id,
                        "x": round(float(x_rob), 1),
                        "y": round(float(y_rob), 1),
                        "z": round(float(z_rob), 1),
                        "cube_center_x": round(float(cx_rob), 1),
                        "cube_center_y": round(float(cy_rob), 1),
                        "cube_center_z": round(float(cz_rob), 1)
                    })
                    
                    cube_centers_data.append([cx_rob, cy_rob, cz_rob])
                    
        if len(cube_centers_data) > 0:
            avg_c = np.mean(cube_centers_data, axis=0)
            avg_cube_data = {"x": round(float(avg_c[0]), 1), "y": round(float(avg_c[1]), 1), "z": round(float(avg_c[2]), 1)}
        else:
            avg_cube_data = None

        aruco_data = {
            "markers": current_data,
            "cube_center": avg_cube_data
        }
        
        return frame, aruco_data


class ArucoClient:
    """Client for fetching live ArUco data securely from stream_server.py"""
    
    def __init__(self, host="127.0.0.1", port=7123):
        self.base_url = f"http://{host}:{port}"
        
    def set_enabled(self, state=True):
        """Enable or disable the background processor on stream_server."""
        val = 1 if state else 0
        try:
            req = urllib.request.Request(f"{self.base_url}/api/aruco/set?enabled={val}")
            with urllib.request.urlopen(req) as response:
                return json.loads(response.read())
        except Exception as e:
            print(f"% ArucoClient Set Error: {e}")
            return None

    def get_data(self):
        """Fetch the latest tracker data processed by stream_server (returns None if failed or disabled)."""
        try:
            req = urllib.request.Request(f"{self.base_url}/api/aruco/data")
            with urllib.request.urlopen(req) as response:
                data = json.loads(response.read())
                if data.get("enabled"):
                    return data
                return None
        except Exception as e:
            return None

# Singleton-style convenience for directly using the class without instantiation every time
processor = None
def get_processor():
    global processor
    if processor is None:
        processor = ArucoProcessor()
    return processor

client = None
def get_client():
    global client
    if client is None:
        client = ArucoClient()
    return client



def find_and_catch():
    """Main mission state machine with two-phase approach for ArUco Cube"""
    import time as t
    import numpy as np
    from uservice import service
    from spose import pose

    state = 0
    arm_reach = 0.32
    approach_margin = 0.30 # stop 20cm before arm reach on first pass
    
    target_distance = 0.0
    target_angle = 0.0
    drive_dist = 0.0
    
    # We will use the global client instance!
    cli = get_client()
    cli.set_enabled(True)

    print("% Starting Cube Catch Mission (from Web!)")
    service.send("robobot/cmd/T0", "leds 16 0 0 100") # Blue LED: Searching
    service.send("robobot/cmd/T0", "servo 1 -900 300") # Ensure arm is UP
    service.stop = False
    
    t.sleep(1.0)
    
    while not service.stop:
        if state == 0: 
            # STATE 0: Look for the cube
            data = cli.get_data()
            if data and data.get("cube_center"):
                service.send("robobot/cmd/T0", "leds 16 0 100 0") # Green LED: Found
                service.send("robobot/cmd/ti", "rc 0.0 0.0") # Stop spinning
                
                # Fetch Coordinates directly from streaming server
                rx = data["cube_center"]["x"] / 1000.0
                ry = data["cube_center"]["y"] / 1000.0
                rz = data["cube_center"]["z"] / 1000.0
                
                target_distance = np.sqrt(rx**2 + ry**2)
                target_angle = np.arctan2(-rx, ry)
                
                drive_dist = target_distance - arm_reach - approach_margin
                
                print(f"% Cube found! Center at: x={rx:.3f} m, y={ry:.3f} m (Z={rz:.3f}m)")
                print(f"%   Distance={target_distance:.3f} m, Angle={np.degrees(target_angle):.1f} deg")
                print(f"%   Phase 1 drive: {drive_dist:.3f} m (dist - arm - margin)")
                
                pose.tripBreset() 
                state = 1
            else:
                # Spin to search
                service.send("robobot/cmd/ti", "rc 0.0 0.4") 
                t.sleep(0.1)
                
        elif state == 1:
            # STATE 1: Turn towards the cube
            if target_angle > 0:
                service.send("robobot/cmd/ti", "rc 0.0 0.5")
            else:
                service.send("robobot/cmd/ti", "rc 0.0 -0.5")
                
            if abs(pose.tripBh) >= abs(target_angle):
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print(f"% Turned {np.degrees(pose.tripBh):.1f} deg (target {np.degrees(target_angle):.1f} deg)")
                pose.tripBreset()
                state = 2
            t.sleep(0.05)

        elif state == 2:
            # STATE 2: Drive towards the cube (phase 1)
            if drive_dist <= 0:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                t.sleep(0.3)
                state = 10
            else:
                service.send("robobot/cmd/ti", "rc 0.1 0.0")
                if pose.tripB >= drive_dist:
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                    t.sleep(0.3)
                    state = 10
            t.sleep(0.05)

        elif state == 10:
            # STATE 10: Re-detect the cube closely
            data = cli.get_data()
            if data and data.get("cube_center"):
                rx = data["cube_center"]["x"] / 1000.0
                ry = data["cube_center"]["y"] / 1000.0
                
                target_distance = np.sqrt(rx**2 + ry**2)
                target_angle = np.arctan2(-rx, ry)
                drive_dist = target_distance - arm_reach
                
                print(f"% Phase 2: Cube at x={rx:.3f} m, y={ry:.3f} m")
                print(f"%   Phase 2 drive: {drive_dist:.3f} m (dist - arm)")
                
                pose.tripBreset()
                state = 11
            else:
                print("% Phase 2: Cube not tracked anymore, catching blind!")
                drive_dist = approach_margin
                pose.tripBreset()
                state = 12
            t.sleep(0.1)

        elif state == 11:
            # STATE 11: Fine-adjust heading
            if abs(target_angle) < 0.05:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                pose.tripBreset()
                state = 12
            else:
                if target_angle > 0:
                    service.send("robobot/cmd/ti", "rc 0.0 0.3")
                else:
                    service.send("robobot/cmd/ti", "rc 0.0 -0.3")
                    
                if abs(pose.tripBh) >= abs(target_angle):
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                    pose.tripBreset()
                    state = 12
            t.sleep(0.05)
            
        elif state == 12:
            # Drop the arm
            service.send("robobot/cmd/T0", "servo 1 10 300")
            print("% Arm dropped")
            state = 13
            t.sleep(1.0)
            
        elif state == 13:
            # Drive the last segment
            if drive_dist <= 0:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                state = 14
            else:
                service.send("robobot/cmd/ti", "rc 0.1 0.0")
                if pose.tripB >= drive_dist:
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                    state = 14
            t.sleep(0.05)
            
        elif state == 14:
            # CATCH
            service.send("robobot/cmd/T0", "servo 1 -900 300")
            service.send("robobot/cmd/T0", "leds 16 0 0 100") 
            print("% Cube presumably caught and lifted!")
            t.sleep(0.5)
            
            cli.set_enabled(False)
            service.stop = True
            break
