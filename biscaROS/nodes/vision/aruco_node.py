"""
The aruco node detects ArUco markers in the camera feed and publishes their positions and orientations to the rest of the system on MQTT. It also provides the data for the visual overlay of the markers in the stream node.
"""

import os
import json
import time
import numpy as np
import cv2 as cv
from multiprocessing import shared_memory
from biscaROS.core.base_node import BaseNode

MARKER_DB = {
    # Static markers
    10: {'type': 'static', 'size': 0.1, 'x': 124.0, 'y': -177.0, 'z': 117.0, 'yaw': np.radians(135)},
    11: {'type': 'static', 'size': 0.1, 'x': 124.0, 'y': 117.0, 'z': 117.0, 'yaw': np.radians(45)},
    12: {'type': 'static', 'size': 0.1, 'x': 117.0, 'y': 124.0, 'z': 117.0, 'yaw': np.radians(-135)},
    13: {'type': 'static', 'size': 0.1, 'x': -117.0, 'y': 124.0, 'z': 117.0, 'yaw': np.radians(135)},
    14: {'type': 'static', 'size': 0.1, 'x': -124.0, 'y': 117.0, 'z': 117.0, 'yaw': np.radians(-45)},
    15: {'type': 'static', 'size': 0.1, 'x': -124.0, 'y': -117.0, 'z': 117.0, 'yaw': np.radians(-135)},
    16: {'type': 'static', 'size': 0.1, 'x': -117.0, 'y': -124.0, 'z': 117.0, 'yaw': np.radians(45)},
    17: {'type': 'static', 'size': 0.1, 'x': 117.0, 'y': -124.0, 'z': 117.0, 'yaw': np.radians(-45)},
    # Cubes
    20: {'type': 'cube', 'size': 0.035, 'cube_size': 60.0},
    53: {'type': 'cube', 'size': 0.035, 'cube_size': 60.0},
    # Carriage
    5: {'type': 'carriage', 'size': 0.035}
}

class ArucoNode(BaseNode):
    def __init__(self):
        super().__init__(node_name="aruco_detector_node")
        
        self.frame_shape = (616, 820, 3)
        self.frame_size = int(np.prod(self.frame_shape))
        
        # 1. Attach to Camera SHM (Input)
        try:
            self.cam_shm = shared_memory.SharedMemory(name="biscaROS_camera_frame")
            self.cam_array = np.ndarray(self.frame_shape, dtype=np.uint8, buffer=self.cam_shm.buf)
        except FileNotFoundError:
            print("[ArucoNode] Error: Camera shared memory not found. Start camera node first.")
            exit(1)
            
        # 2. Create Overlay SHM (Output)
        try:
            self.overlay_shm = shared_memory.SharedMemory(name="biscaROS_aruco_overlay", create=True, size=self.frame_size)
        except FileExistsError:
            self.overlay_shm = shared_memory.SharedMemory(name="biscaROS_aruco_overlay", create=False)
            
        self.overlay_array = np.ndarray(self.frame_shape, dtype=np.uint8, buffer=self.overlay_shm.buf)
        
        # 3. Vision specific initialization
        self.mtx, self.dist = self._load_calibration()
        self.R_cam2rob, self.T_cam2rob = self._get_extrinsic_matrix()
        
        self.aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
        if hasattr(cv.aruco, 'DetectorParameters_create'):
            self.aruco_params = cv.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv.aruco.DetectorParameters()
            
        self.client.subscribe("biscaROS/vision/camera/new_frame")

    def _load_calibration(self):
        # Get the directory of aruco_node.py (nodes/vision)
        current_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Traverse up two levels and into the core directory
        core_dir = os.path.abspath(os.path.join(current_dir, "..", "..", "core"))
        
        calib_file = os.path.join(core_dir, "calibration_checkerboard.npz")
        fallback_file = os.path.join(core_dir, "calibration_data.npz")
        
        mtx, dist = None, None
        
        if os.path.exists(calib_file):
            print(f"[ArucoNode] Loading calibration from: {calib_file}", flush=True)
            with np.load(calib_file) as X:
                mtx, dist = [X[i] for i in ('mtx', 'dist')]
        elif os.path.exists(fallback_file):
            print(f"[ArucoNode] Loading fallback calibration from: {fallback_file}", flush=True)
            with np.load(fallback_file) as X:
                mtx, dist = [X[i] for i in ('mtx', 'dist')]
        else:
            print(f"[ArucoNode] ERROR: No calibration files found in {core_dir}", flush=True)
            
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

    def _on_message(self, client, userdata, msg):
        # print(f"[ArucoNode] Triggered by {msg.topic}", flush=True)
        # print(f"[ArucoNode] Matrix: {self.mtx is not None}, Dist: {self.dist is not None}", flush=True)

        # if msg.topic != "biscaROS/vision/camera/new_frame" or self.mtx is None or self.dist is None:
        #     print("[ArucoNode] Guard condition failed. Aborting frame.", flush=True)
        #     return

        if msg.topic != "biscaROS/vision/camera/new_frame" or self.mtx is None or self.dist is None:
            return

        # Copy frame and create black canvas for isolated overlays
        frame_bgr = np.copy(self.cam_array)
        overlay_bgr = np.zeros_like(frame_bgr)
        gray = cv.cvtColor(frame_bgr, cv.COLOR_BGR2GRAY)
        
        try:
            detector = cv.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            corners, ids, _ = detector.detectMarkers(gray)
        except AttributeError:
            corners, ids, _ = cv.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        current_data = []
        cube_centers_data = []
        estimates = []

        if ids is not None:
            # Draw markers on the BLACK CANVAS, not the camera frame
            cv.aruco.drawDetectedMarkers(overlay_bgr, corners, ids)
            
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
                    # Draw axes on the BLACK CANVAS
                    cv.drawFrameAxes(overlay_bgr, self.mtx, self.dist, rvec, tvec, size)
                    
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
                        R_rob2marker = self.R_cam2rob @ R_marker
                        T_rob2marker = marker_rob 
                        
                        sy = np.sqrt(R_rob2marker[0,0] * R_rob2marker[0,0] +  R_rob2marker[1,0] * R_rob2marker[1,0])
                        if not (sy < 1e-6):
                            m_roll = np.arctan2(R_rob2marker[2,1], R_rob2marker[2,2])
                            m_pitch = np.arctan2(-R_rob2marker[2,0], sy)
                            m_yaw = np.arctan2(R_rob2marker[1,0], R_rob2marker[0,0])
                        else:
                            m_roll = np.arctan2(-R_rob2marker[1,2], R_rob2marker[1,1])
                            m_pitch = np.arctan2(-R_rob2marker[2,0], sy)
                            m_yaw = 0
                            
                        marker_obj["yaw"] = round(float(m_yaw), 2)
                        marker_obj["pitch"] = round(float(m_pitch), 2)
                        marker_obj["roll"] = round(float(m_roll), 2)
                        
                        world_x, world_y, world_z, world_yaw = marker_info['x'], marker_info['y'], marker_info['z'], marker_info['yaw']
                        T_world2marker = np.array([[world_x], [world_y], [world_z]])
                        
                        R_roll = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
                        R_yaw_mat = np.array([
                            [np.cos(world_yaw), -np.sin(world_yaw), 0],
                            [np.sin(world_yaw),  np.cos(world_yaw), 0],
                            [0, 0, 1]
                        ])
                        R_world2marker = R_yaw_mat @ R_roll
                        
                        R_world2rob = R_world2marker @ R_rob2marker.T
                        T_world2rob = T_world2marker - (R_world2rob @ T_rob2marker)
                        
                        robot_x_est = T_world2rob[0, 0]
                        robot_y_est = T_world2rob[1, 0]
                        robot_z_est = 0
                        
                        sy = np.sqrt(R_world2rob[0,0] * R_world2rob[0,0] +  R_world2rob[1,0] * R_world2rob[1,0])
                        if not (sy < 1e-6):
                            robot_yaw_est = np.arctan2(R_world2rob[1,0], R_world2rob[0,0])
                        else:
                            robot_yaw_est = 0
                            
                        estimates.append({
                            "marker_id": marker_id,
                            "robot_x": round(float(robot_x_est), 1),
                            "robot_y": round(float(robot_y_est), 1),
                            "robot_z": round(float(robot_z_est), 1),
                            "robot_roll": 0.0,
                            "robot_pitch": 0.0,
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

        # Publish math results
        self.publish("biscaROS/vision/aruco/data", json.dumps(aruco_data))
        
        # Convert black canvas overlay back to RGB and save to shared memory
        overlay_rgb = cv.cvtColor(overlay_bgr, cv.COLOR_BGR2RGB)
        np.copyto(self.overlay_array, overlay_rgb)
        self.publish("biscaROS/vision/overlays/ready", "biscaROS_aruco_overlay")

    def stop(self):
        self.overlay_shm.close()
        self.overlay_shm.unlink()
        super().stop()

if __name__ == "__main__":
    node = ArucoNode()
    node.start()
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        node.stop()