import time as t
import numpy as np
import cv2 as cv
import os

from uservice import service
from scam import cam
import aruco

def find_and_print():
    """
    Takes a single picture, finds ArUco markers using the shared ArucoProcessor,
    saves an annotated image, and prints the coordinates.
    """
    print("% Starting Shared ArUco Pose Estimation Test!")
    os.makedirs("aruco_test_results", exist_ok=True)
    
    processor = aruco.get_processor()
    if processor.mtx is None:
        print("% ERROR: Calibration file not found. Please run calibration first.")
        service.stop = True
        return
    
    while not service.stop:
        ok, img, imgTime = cam.getImage()
        
        if ok:
            timestamp = imgTime.strftime('%Y%m%d_%H%M%S')
            
            # Using shared logic!
            annotated_img, data = processor.process_image(img.copy())
            
            markers = data.get("markers", [])
            
            if len(markers) > 0:
                print("\n" + "="*40)
                print(f"       {len(markers)} ARUCO MARKER(S) FOUND!")
                print("="*40)
                
                for m in markers:
                    print(f"Marker ID : {m['id']}")
                    print(f"Robot Pose: X = {m['x']:.1f} mm (right)")
                    print(f"            Y = {m['y']:.1f} mm (forward)")
                    print(f"            Z = {m['z']:.1f} mm (up)")
                    print(f"Cube Cntr : X = {m['cube_center_x']:.1f} mm")
                    print(f"            Y = {m['cube_center_y']:.1f} mm")
                    print(f"            Z = {m['cube_center_z']:.1f} mm")
                    print("-" * 40)
                    
                target = data.get("cube_center")
                if target:
                    print(f"Averaged Cube Center: X={target['x']:.1f}, Y={target['y']:.1f}, Z={target['z']:.1f}")
                
                filename = f"aruco_test_results/{timestamp}_aruco_pose.jpg"
                cv.imwrite(filename, annotated_img)
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
