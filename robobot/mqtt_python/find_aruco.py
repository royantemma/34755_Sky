import time as t
import numpy as np
import cv2 as cv
import os

from uservice import service
from scam import cam

def find_and_print():
    """
    Takes a single picture, finds a 4x4 ArUco marker,
    estimates its pose relative to the camera using the calibration data,
    saves an annotated image, and prints the coordinates.
    """
    print("% Starting ArUco Pose Estimation Test!")
    
    # Try to load calibration data
    calib_file = "calibration_checkerboard.npz"
    if not os.path.exists(calib_file):
        calib_file = "calibration_data.npz" # Fallback
        if not os.path.exists(calib_file):
            print("% ERROR: Calibration file not found. Please run calibration first.")
            service.stop = True
            return
            
    print(f"% Loading calibration from {calib_file}")
    with np.load(calib_file) as X:
        mtx, dist = [X[i] for i in ('mtx', 'dist')]
        
    os.makedirs("aruco_test_results", exist_ok=True)
    
    marker_size = 0.035 # 35mm in meters
    
    # Define the 3D coordinates of the marker corners
    obj_points = np.array([
        [-marker_size/2,  marker_size/2, 0],
        [ marker_size/2,  marker_size/2, 0],
        [ marker_size/2, -marker_size/2, 0],
        [-marker_size/2, -marker_size/2, 0]
    ], dtype=np.float32)

    # ArUco dictionary (4x4)
    aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
    
    # Handle OpenCV version differences for Detector parameters
    if hasattr(cv.aruco, 'DetectorParameters_create'):
        aruco_params = cv.aruco.DetectorParameters_create()
    else:
        aruco_params = cv.aruco.DetectorParameters()
    
    while not service.stop:
        ok, img, imgTime = cam.getImage()
        
        if ok:
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            
            # Handle OpenCV version differences for detecting markers
            try:
                # Modern OpenCV 4.7+
                detector = cv.aruco.ArucoDetector(aruco_dict, aruco_params)
                corners, ids, rejected = detector.detectMarkers(gray)
            except AttributeError:
                # Older OpenCV
                corners, ids, rejected = cv.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
            
            timestamp = imgTime.strftime('%Y%m%d_%H%M%S')
            
            if ids is not None and len(ids) > 0:
                print("\n" + "="*40)
                print(f"       ARUCO MARKER(S) FOUND!")
                print("="*40)
                
                # Draw detected markers
                cv.aruco.drawDetectedMarkers(img, corners, ids)
                
                for i in range(len(ids)):
                    marker_id = ids[i][0]
                    marker_corners = corners[i][0]
                    
                    # Estimate pose
                    ret, rvec, tvec = cv.solvePnP(obj_points, marker_corners, mtx, dist)
                    
                    if ret:
                        # Draw axis
                        cv.drawFrameAxes(img, mtx, dist, rvec, tvec, marker_size)
                        
                        x, y, z = tvec.flatten()
                        
                        # Convert variables to mm for printing
                        x_mm, y_mm, z_mm = x * 1000, y * 1000, z * 1000
                        
                        # Calculate robot frame coordinates
                        # Camera is rolled down by 5.37 degrees on X axis
                        # From CAD : 11.4 deg, 184mm
                        # Experimentally measured: 5.37 deg, 189.8mm
                        theta = np.radians(5.37)
                        cam_z_offset = 189.8 # 189.8 mm from origin
                        
                        # Apply rotation and translation to match the origin
                        # OpenCV camera frame is Z=forward, X=right, Y=down
                        # Robot frame usually: Y=forward, X=right, Z=up
                        x_rob = x_mm
                        y_rob = z_mm * np.cos(theta) - y_mm * np.sin(theta)
                        z_rob = cam_z_offset - (y_mm * np.cos(theta) + z_mm * np.sin(theta))
                        
                        print(f"Marker ID : {marker_id}")
                        print(f"Cam Pose  : X = {x_mm:.1f} mm (right)")
                        print(f"            Y = {y_mm:.1f} mm (down)")
                        print(f"            Z = {z_mm:.1f} mm (forward)")
                        print(f"Robot Pose: X = {x_rob:.1f} mm (right)")
                        print(f"            Y = {y_rob:.1f} mm (forward)")
                        print(f"            Z = {z_rob:.1f} mm (up)")
                        print("-" * 40)
                
                filename = f"aruco_test_results/{timestamp}_aruco_pose.jpg"
                cv.imwrite(filename, img)
                print(f"  -> Saved {filename}\n")
            else:
                print("\n" + "="*40)
                print("       NO ARUCO MARKER FOUND")
                print("="*40 + "\n")
                
                filename = f"aruco_test_results/{timestamp}_no_aruco.jpg"
                cv.imwrite(filename, img)
                print(f"  -> Saved {filename} (No markers)\n")
                
            service.stop = True
            break
            
        t.sleep(0.1)
