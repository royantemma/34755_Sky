import sys

content = """
import numpy as np
import cv2 as cv
import os
import urllib.request
import json
import time

MARKER_DB = {
    # Static markers
    10: {'type': 'static', 'size': 0.1, 'x': 1240.0, 'y': -1770.0, 'z': 117.0, 'yaw': np.radians(45)},
    11: {'type': 'static', 'size': 0.1, 'x': 1240.0, 'y': 1170.0, 'z': 117.0, 'yaw': np.radians(-45)},
    12: {'type': 'static', 'size': 0.1, 'x': 1170.0, 'y': 1240.0, 'z': 117.0, 'yaw': np.radians(135)},
    13: {'type': 'static', 'size': 0.1, 'x': -1170.0, 'y': 1240.0, 'z': 117.0, 'yaw': np.radians(45)},
    14: {'type': 'static', 'size': 0.1, 'x': -1240.0, 'y': 1170.0, 'z': 117.0, 'yaw': np.radians(225)},
    15: {'type': 'static', 'size': 0.1, 'x': -1240.0, 'y': -1170.0, 'z': 117.0, 'yaw': np.radians(135)},
    16: {'type': 'static', 'size': 0.1, 'x': -1170.0, 'y': -1240.0, 'z': 117.0, 'yaw': np.radians(-45)},
    17: {'type': 'static', 'size': 0.1, 'x': 1170.0, 'y': -1240.0, 'z': 117.0, 'yaw': np.radians(225)},
    # Cubes
    20: {'type': 'cube', 'size': 0.035, 'cube_size': 60.0},
    53: {'type': 'cube', 'size': 0.035, 'cube_size': 60.0},
    # Carriage
    5: {'type': 'carriage', 'size': 0.035}
}

class ArucoProcessor:
    def __init__(self):
        self.mtx, self.dist = self._load_calibration()
        self.R_cam2rob, self.T_cam2rob = self._get_extrinsic_matrix()
        
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
        theta = np.radians(20.09) 
        phi = np.radians(1.4)     
        cam_z_offset = 236.9
        cam_y_offset = 20.0       
        
        R_yaw = np.array([
            [np.cos(phi), 0, np.sin(phi)],
            [0, 1, 0],
            [-np.sin(phi), 0, np.cos(phi)]
        ])
        
        R_pitch_base = np.array([
            [1, 0, 0],
            [0, -np.sin(theta), np.cos(theta)],
            [0, -np.cos(theta), -np.sin(theta)]
        ])
        
        R_cam2rob = R_pitch_base @ R_yaw
        T_cam2rob = np.array([[0.0], [cam_y_offset], [cam_z_offset]])
        return R_cam2rob, T_cam2rob

    def _get_obj_points(self, size):
        return np.array([
            [-size/2,  size/2, 0],
            [ size/2,  size/2, 0],
            [ size/2, -size/2, 0],
            [-size/2, -size/2, 0]
        ], dtype=np.float32)

    def process_image(self, frame, valid_ids=None):
        if self.mtx is None or self.dist is None:
            return frame, {"markers": [], "cube_center": None, "estimates": []}
            
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        try:
            detector = cv.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            corners, ids, _ = detector.detectMarkers(gray)
        except AttributeError:
            corners, ids, _ = cv.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        current_data = [] # marker objects
        cube_centers_data = []
        estimates = [] # position estimates

        if ids is not None:
            cv.aruco.drawDetectedMarkers(frame, corners, ids)
            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                if marker_id not in MARKER_DB:
                    continue
                
                marker_info = MARKER_DB[marker_id]
                size = marker_info['size']
                m_type = marker_info['type']
                obj_pts = self._get_obj_points(size)
                
                marker_corners = corners[i][0]
                ret, rvec, tvec = cv.solvePnP(obj_pts, marker_corners, self.mtx, self.dist)
                if ret:
                    cv.drawFrameAxes(frame, self.mtx, self.dist, rvec, tvec, size)
                    
                    tvec_mm = tvec * 1000.0
                    R_marker, _ = cv.Rodrigues(rvec)
                    marker_rob = (self.R_cam2rob @ tvec_mm) + self.T_cam2rob
                    x_rob, y_rob, z_rob = marker_rob.flatten()
                    
                    marker_obj = {
                        "id": marker_id,
                        "type": m_type,
                        "x": round(float(x_rob), 1),
                        "y": round(float(y_rob), 1),
                        "z": round(float(z_rob), 1)
                    }
                    current_data.append(marker_obj)
                    
                    if m_type == 'cube':
                        cs = marker_info['cube_size']
                        offset_marker_frame = np.array([[0.0], [0.0], [-cs/2.0]])
                        cube_center_cam_mm = tvec_mm + (R_marker @ offset_marker_frame)
                        cube_rob = (self.R_cam2rob @ cube_center_cam_mm) + self.T_cam2rob
                        cx_rob, cy_rob, cz_rob = cube_rob.flatten()
                        marker_obj["cube_cx"] = round(float(cx_rob), 1)
                        marker_obj["cube_cy"] = round(float(cy_rob), 1)
                        marker_obj["cube_cz"] = round(float(cz_rob), 1)
                        cube_centers_data.append([cx_rob, cy_rob, cz_rob])
                        
                    elif m_type == 'static':
                        # Compute robot position
                        world_x, world_y, world_z, world_yaw = marker_info['x'], marker_info['y'], marker_info['z'], marker_info['yaw']
                        
                        R_rob2cam = self.R_cam2rob.T
                        T_rob2cam = -R_rob2cam @ self.T_cam2rob
                        
                        R_world2cam = R_marker
                        T_world2cam = tvec_mm
                        
                        # We want R_world2rob, T_world2rob to figure out where robot is in world.
                        R_world2rob = self.R_cam2rob @ R_world2cam
                        T_world2rob = (self.R_cam2rob @ T_world2cam) + self.T_cam2rob
                        
                        R_rob2world = R_world2rob.T
                        T_rob2world = -R_rob2world @ T_world2rob
                        
                        robot_x_est = world_x + T_rob2world[0,0]
                        robot_y_est = world_y + T_rob2world[1,0]
                        robot_yaw_est = world_yaw + np.arctan2(R_rob2world[1,0], R_rob2world[0,0])
                        
                        estimates.append({
                            "marker_id": marker_id,
                            "robot_x": round(float(robot_x_est), 1),
                            "robot_y": round(float(robot_y_est), 1),
                            "robot_yaw": float(robot_yaw_est)
                        })

        avg_cube_data = None
        if len(cube_centers_data) > 0:
            avg_c = np.mean(cube_centers_data, axis=0)
            avg_cube_data = {"x": round(float(avg_c[0]), 1), "y": round(float(avg_c[1]), 1), "z": round(float(avg_c[2]), 1)}

        aruco_data = {
            "markers": current_data,
            "cube_center": avg_cube_data,
            "estimates": estimates
        }
        
        return frame, aruco_data

class ArucoClient:
    def __init__(self, host="127.0.0.1", port=7123):
        self.base_url = f"http://{host}:{port}"
        
    def set_enabled(self, state=True):
        val = 1 if state else 0
        try:
            req = urllib.request.Request(f"{self.base_url}/api/aruco/set?enabled={val}")
            with urllib.request.urlopen(req) as response:
                return json.loads(response.read())
        except Exception as e:
            print(f"% ArucoClient Set Error: {e}")
            return None

    def get_data(self):
        try:
            req = urllib.request.Request(f"{self.base_url}/api/aruco/data")
            with urllib.request.urlopen(req) as response:
                data = json.loads(response.read())
                if data.get("enabled"):
                    return data
                return None
        except Exception as e:
            return None

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

"""

with open("mqtt_python/aruco.py", "w") as f:
    f.write(content)
    
print("Updated aruco.py successfully")
