import cv2 as cv
import numpy as np
import time as t
import os
import glob

# Import framework modules
from uservice import service
from scam import cam

def run_calibration():
    """
    Takes 1 picture every 3 seconds, 30 times, and saves them.
    Then uses OpenCV to find the camera calibration matrix from the checkerboard images.
    """
    print("% Starting Checkboard Calibration Image Capture!")
    folder_path = "calibrate_checkerboard_images"
    os.makedirs(folder_path, exist_ok=True)
    
    num_images = 30
    delay_between_images = 3.0
    images_taken = 0
    
    # Wait for camera to be ready
    t.sleep(1.0)
    
    while images_taken < num_images and not service.stop:
        ok, img, imgTime = cam.getImage()
        
        if ok:
            timestamp = imgTime.strftime('%Y%m%d_%H%M%S')
            filename = f"{folder_path}/checkerboard_{images_taken:02d}_{timestamp}.jpg"
            
            # Show a message that we are capturing
            print(f"% Capturing image {images_taken + 1}/{num_images} -> {filename}")
            cv.imwrite(filename, img)
            
            images_taken += 1
            
            # Wait 3 seconds before next capture, but handle service.stop
            start_time = t.time()
            while t.time() - start_time < delay_between_images and not service.stop:
                t.sleep(0.1)
        else:
            t.sleep(0.1)
            
    if service.stop:
        print("% Calibration interrupted.")
        return

    print("% Image capture complete. Starting calibration...")
    
    # Checkerboard dimensions
    # Note: OpenCV expects the number of inner corners. Board has 11x8 squares, the inner corners are 10x7.
    checkerboard_size = (10, 7)
    square_size = 20.0 # mm
    
    # Prepare object points, like (0,0,0), (20,0,0), (40,0,0) ...
    objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2) * square_size
    
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    
    images = glob.glob(f'{folder_path}/*.jpg')
    
    if not images:
        print("% Error: No images found for calibration.")
        return
        
    gray = None
    success_count = 0
    
    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, checkerboard_size, None)
        
        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)
            
            # Define criteria for subpixel accuracy
            criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            
            # Draw and display the corners (optional, can be saved instead)
            cv.drawChessboardCorners(img, checkerboard_size, corners2, ret)
            cv.imwrite(fname.replace('.jpg', '_corners.jpg'), img)
            success_count += 1
            print(f"% Found corners in {fname}")
        else:
            print(f"% Could not find corners in {fname}")
            
    print(f"% Found corners in {success_count}/{len(images)} images.")
    
    if success_count > 0:
        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        
        print("\n" + "="*40)
        print("CAMERA CALIBRATION RESULTS")
        print("="*40)
        print(f"RMS Reprojection Error: {ret:.4f}")
        print("Camera Matrix (mtx):")
        print(mtx)
        print("\nDistortion Coefficients (dist):")
        print(dist)
        print("="*40 + "\n")
        
        # Save results
        np.savez("calibration_data.npz", mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
        print("% Saved calibration results to calibration_data.npz")
    else:
        print("% Failed to calibrate. Could not find checkerboard in any images.")
