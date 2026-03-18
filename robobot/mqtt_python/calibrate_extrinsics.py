import time
import numpy as np
import cv2 as cv
import os

from uservice import service
from scam import cam

def run_calibration():
    print("% Starting Extrinsic Camera Calibration!")
    
    # Try to load intrinsic calibration data
    calib_file = "calibration_checkerboard.npz"
    if not os.path.exists(calib_file):
        calib_file = "calibration_data.npz"
        if not os.path.exists(calib_file):
            print("% ERROR: Calibration file not found. Please run checkerboard calibration first.")
            service.stop = True
            return
            
    with np.load(calib_file) as X:
        mtx, dist = [X[i] for i in ('mtx', 'dist')]
        
    marker_size = 0.035 # 35mm marker
    
    # 3D points of ArUco
    obj_points = np.array([
        [-marker_size/2,  marker_size/2, 0],
        [ marker_size/2,  marker_size/2, 0],
        [ marker_size/2, -marker_size/2, 0],
        [-marker_size/2, -marker_size/2, 0]
    ], dtype=np.float32)

    # Handle OpenCV versions
    aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
    if hasattr(cv.aruco, 'DetectorParameters_create'):
        aruco_params = cv.aruco.DetectorParameters_create()
    else:
        aruco_params = cv.aruco.DetectorParameters()
        
    # ArUco center height from the floor (Assuming a 60mm cube)
    height_of_marker_center_mm = 60.0 / 2.0
        
    print("\n" + "="*60)
    print("      EXTRINSIC CAMERA CALIBRATION ROUTINE")
    print("="*60)
    print("1. Place your 60mm ArUco cube on the floor.")
    print("   Keep it roughly centered in front of the camera.")
    print("2. When recording starts, SLOWLY slide the cube")
    print("   away from the robot (e.g., from 10cm out to 40cm).")
    print("3. Do not lift the cube. Keep it perfectly flat.")
    print("4. Recording will last for 20 seconds.")
    print("="*60)
    print("Starting in 5 seconds...\n")
    time.sleep(5)
    
    print("% RECORDING NOW! Slowly slide the cube back and forth...")
    
    y_points = []
    z_points = []
    
    start_time = time.time()
    last_print = time.time()
    
    while time.time() - start_time < 20 and not service.stop:
        ok, img, _ = cam.getImage()
        if ok:
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            try:
                detector = cv.aruco.ArucoDetector(aruco_dict, aruco_params)
                corners, ids, _ = detector.detectMarkers(gray)
            except AttributeError:
                corners, ids, _ = cv.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
            
            if ids is not None and len(ids) > 0:
                marker_corners = corners[0][0]
                ret, rvec, tvec = cv.solvePnP(obj_points, marker_corners, mtx, dist)
                
                if ret:
                    x, y, z = tvec.flatten()
                    # Append strictly the raw camera frame data
                    y_points.append(y * 1000)
                    z_points.append(z * 1000)
        
        # Give progress output every half second
        if time.time() - last_print > 0.5:
            print(f" -> Recording... {len(y_points)} valid poses collected.")
            last_print = time.time()
            
        time.sleep(0.03) # Limit framerate
        
    if len(y_points) < 15:
        print("\n% ERROR: Not enough data points. Try keeping the ArUco in frame more consistently.")
        service.stop = True
        return
        
    # The Math:
    # We know that height is constant.
    # z_up = H_cam - (y_cam*cos(theta) + z_cam*sin(theta))
    # y_cam = z_cam*(-tan(theta)) + (H_cam - z_up)/cos(theta)
    # This is a linear equation: Y = mx + b
    
    m, b = np.polyfit(z_points, y_points, 1)
    
    theta_rad = -np.arctan(m)
    theta_deg = np.degrees(theta_rad)
    h_cam_mm = (b * np.cos(theta_rad)) + height_of_marker_center_mm
    
    print("\n" + "="*50)
    print("            CALIBRATION COMPLETE!")
    print("="*50)
    print(f" Points recorded : {len(y_points)}")
    print(f" True Pitch Angle: {theta_deg:.2f} degrees")
    print(f" True Z-Height   : {h_cam_mm:.1f} mm")
    print("="*50)
    print("Update `find_aruco.py` with these exact values to eliminate the draft!\n")
    
    service.stop = True