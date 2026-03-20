import time as t
import numpy as np
import cv2 as cv
import os

# Import framework modules
from uservice import service
from scam import cam
from spose import pose

def get_calibration():
    calib_file = "calibration_checkerboard.npz"
    if not os.path.exists(calib_file):
        calib_file = "calibration_data.npz"
        if not os.path.exists(calib_file):
            return None, None
    with np.load(calib_file) as X:
        return X['mtx'], X['dist']

def detect_cube_center_id53(img, mtx, dist):
    """
    Finds ID 53 ArUco marker(s), estimates pose, calculates the center
    30mm along the marker's X-axis, and returns the averaged robot pose.
    Returns: bool_found, target_distance, target_angle, x_rob, y_rob, z_rob
    """
    marker_size = 0.035
    obj_points = np.array([
        [-marker_size/2,  marker_size/2, 0],
        [ marker_size/2,  marker_size/2, 0],
        [ marker_size/2, -marker_size/2, 0],
        [-marker_size/2, -marker_size/2, 0]
    ], dtype=np.float32)

    aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
    if hasattr(cv.aruco, 'DetectorParameters_create'):
        aruco_params = cv.aruco.DetectorParameters_create()
    else:
        aruco_params = cv.aruco.DetectorParameters()
        
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    try:
        detector = cv.aruco.ArucoDetector(aruco_dict, aruco_params)
        corners, ids, _ = detector.detectMarkers(gray)
    except AttributeError:
        corners, ids, _ = cv.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    if ids is None or len(ids) == 0:
        return False, 0.0, 0.0, 0.0, 0.0, 0.0

    cube_centers_rob = []
    
    # Camera Extrinsics
    theta = np.radians(5.36)
    cam_z_offset = 189.2
    
    # Offset of the cube center from the marker surface (measured in meters)
    # The user noted "30mm (x axis) behind the center"
    offset_marker_frame = np.array([[0.030], [0.0], [0.0]]) # Or negative if it's the other direction

    for i in range(len(ids)):
        if ids[i][0] == 53:
            ret, rvec, tvec = cv.solvePnP(obj_points, corners[i][0], mtx, dist)
            if ret:
                # Apply the offset to get the Cube Center in camera frame
                R, _ = cv.Rodrigues(rvec)
                offset_cam = R @ offset_marker_frame
                cube_tvec = tvec.flatten() + offset_cam.flatten()
                
                cx, cy, cz = cube_tvec
                cx_mm, cy_mm, cz_mm = cx * 1000, cy * 1000, cz * 1000
                
                # Transform to Robot Frame (Origin on ground)
                x_rob = cx_mm
                y_rob = cz_mm * np.cos(theta) - cy_mm * np.sin(theta)
                z_rob = cam_z_offset - (cy_mm * np.cos(theta) + cz_mm * np.sin(theta))
                
                # Convert back to meters for driving functions
                cube_centers_rob.append([x_rob / 1000.0, y_rob / 1000.0, z_rob / 1000.0])

    if len(cube_centers_rob) == 0:
        return False, 0.0, 0.0, 0.0, 0.0, 0.0

    # Mean of all 53-id positions
    mean_pose = np.mean(cube_centers_rob, axis=0)
    real_x, real_y, real_z = mean_pose
    
    target_distance = np.sqrt(real_x**2 + real_y**2)
    # arctan2(y_arg, x_arg) -> robot forward is real_y, left is -real_x
    target_angle = np.arctan2(-real_x, real_y)
    
    return True, target_distance, target_angle, real_x, real_y, real_z


def find_and_catch():
    """Main mission state machine with two-phase approach for ArUco Cube ID 53"""
    state = 0
    arm_reach = 0.26 # middle of the cup is 26cm from the center of the robot
    approach_margin = 0.20 # stop 20cm before arm reach on first pass
    
    real_x = 0.0
    real_y = 0.0
    target_distance = 0.0
    target_angle = 0.0
    drive_dist = 0.0
    
    mtx, dist = get_calibration()
    if mtx is None:
        print("% ERROR: Calibration not found for cube catching.")
        return

    print("% Starting Cube Catch Mission (ArUco 53)!")
    service.send("robobot/cmd/T0", "leds 16 0 0 100") # Blue LED: Searching
    service.send("robobot/cmd/T0", "servo 1 -900 300") # Ensure arm is UP
    
    t.sleep(1.0)
    
    while not service.stop:
        if state == 0: 
            # STATE 0: Look for the cube
            ok, img, imgTime = cam.getImage()
            if ok:
                found, dist_tgt, angle_tgt, rx, ry, rz = detect_cube_center_id53(img, mtx, dist)
                if found:
                    service.send("robobot/cmd/T0", "leds 16 0 100 0") # Green LED: Found
                    service.send("robobot/cmd/ti", "rc 0.0 0.0") # Stop spinning
                    
                    target_distance, target_angle = dist_tgt, angle_tgt
                    drive_dist = target_distance - arm_reach - approach_margin
                    
                    print(f"% Cube 53 found! Center at: x={rx:.3f} m, y={ry:.3f} m (Z={rz:.3f}m)")
                    print(f"%   Distance={target_distance:.3f} m, Angle={np.degrees(target_angle):.1f} deg")
                    print(f"%   Phase 1 drive: {drive_dist:.3f} m (dist - arm - margin)")
                    
                    pose.tripBreset() 
                    state = 1
                else:
                    # Spin to search
                    service.send("robobot/cmd/ti", "rc 0.0 0.4") 
                    
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

        elif state == 2:
            # STATE 2: Drive towards the cube (phase 1)
            if drive_dist <= 0:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                t.sleep(0.3)
                state = 10
            else:
                service.send("robobot/cmd/ti", "rc 0.2 0.0")
                if pose.tripB >= drive_dist:
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                    t.sleep(0.3)
                    state = 10

        elif state == 10:
            # STATE 10: Re-detect the cube closely
            ok, img, imgTime = cam.getImage()
            if ok:
                found, dist_tgt, angle_tgt, rx, ry, rz = detect_cube_center_id53(img, mtx, dist)
                if found:
                    target_distance, target_angle = dist_tgt, angle_tgt
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

        elif state == 11:
            # STATE 11: Fine-adjust heading
            if abs(target_angle) < 0.05:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                pose.tripBreset()
                state = 12
            else:
                if target_angle > 0:
                    service.send("robobot/cmd/ti", "rc 0.0 0.4")
                else:
                    service.send("robobot/cmd/ti", "rc 0.0 -0.4")
                if abs(pose.tripBh) >= abs(target_angle):
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                    pose.tripBreset()
                    state = 12

        elif state == 12:
            # STATE 12: Drive final stretch
            if drive_dist <= 0:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                state = 3
            else:
                service.send("robobot/cmd/ti", "rc 0.15 0.0") 
                if pose.tripB >= drive_dist:
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                    state = 3
                    
        elif state == 3:
            # STATE 3: Catch
            print("% Catching the cube!")
            service.send("robobot/cmd/T0", "servo 1 100 400") 
            t.sleep(1.5) 
            state = 99
            
        elif state == 99:
            # STATE 99: Finished
            print("% Mission Accomplished.")
            service.send("robobot/cmd/T0", "leds 16 100 0 100") # Purple LED
            break
            
        t.sleep(0.05)