import time as t
import numpy as np
import cv2 as cv
import os

# Import framework modules
from uservice import service
from scam import cam
from spose import pose

def detect_red_ball(img):
    """
    Converts image to HSV, masks red pixels, finds the ball, 
    and returns a dictionary of all intermediate images for debugging.
    """
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    
    # Red wraps around the 0/180 mark in OpenCV
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    
    # Step 1: Individual Masks
    mask1 = cv.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv.inRange(hsv, lower_red2, upper_red2)
    
    # Step 2: Combined Mask
    combined_mask = cv.bitwise_or(mask1, mask2)
    
    # Step 3: Morphological Cleanup
    eroded_mask = cv.erode(combined_mask, None, iterations=2)
    final_mask = cv.dilate(eroded_mask, None, iterations=2)
    
    # Pack all steps into a dictionary to send back to the test function
    debug_images = {
        "01_original": img.copy(),
        "02_mask1_low_red": mask1,
        "03_mask2_high_red": mask2,
        "04_combined_mask": combined_mask,
        "05_final_cleaned_mask": final_mask
    }
    
    # Find Contours
    contours, _ = cv.findContours(final_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    if len(contours) > 0:
        largest_contour = max(contours, key=cv.contourArea)
        ((x, y), radius) = cv.minEnclosingCircle(largest_contour)
        
        if radius > 5: 
            # Draw a green circle and a red dot at the center on the debug image
            cv.circle(debug_images["01_original"], (int(x), int(y)), int(radius), (0, 255, 0), 2)
            cv.circle(debug_images["01_original"], (int(x), int(y)), 3, (0, 0, 255), -1)
            
            return True, int(x), int(y), radius, debug_images
            
    return False, 0, 0, 0, debug_images

def setup_homography(img_width, img_height):
    """
    Calculates the Perspective Transform matrix using CAD projection data.
    """
    # 1. Pixel coordinates (Source)
    # Mapping: Middle Left, Middle Right, Bottom Left, Bottom Right
    # src_pts = np.array([
    #     [0, img_height / 2.0],           # Middle left pixel
    #     [img_width, img_height / 2.0],   # Middle right pixel
    #     [0, img_height],                 # Bottom left pixel
    #     [img_width, img_height]          # Bottom right pixel
    # ], dtype=np.float32)
    src_pts = np.array([ # For real measurement of the ball
        [213, 304],           # Middle left pixel
        [601, 307],   # Middle right pixel
        [45, 431],                 # Bottom left pixel
        [774, 437]          # Bottom right pixel
    ], dtype=np.float32)

    # 2. Real-world CAD coordinates (Destination) converted from mm to METERS
    # X: Left is negative, Right is positive
    # Y: Forward distance from robot
    # dst_pts = np.array([ # For z=0mm
    #     [-0.573709, 0.9291],   # Middle left CAD
    #     [0.573709, 0.9291],    # Middle right CAD
    #     [-0.176576, 0.257516], # Bottom left CAD
    #     [0.176576, 0.257516]   # Bottom right CAD
    # ], dtype=np.float32)
    # dst_pts = np.array([ # For z=21mm (middle of the ball)
    #     [-0.510, 0.925],   # Middle left CAD
    #     [0.510, 0.925],    # Middle right CAD
    #     [-0.157, 0.228], # Bottom left CAD
    #     [0.157, 0.228]   # Bottom right CAD
    # ], dtype=np.float32)
    dst_pts = np.array([ # For real measurement of the ball
        [-0.52, 1.75],   # Middle left CAD
        [0.52, 1.75],    # Middle right CAD
        [-0.31, 0.55], # Bottom left CAD
        [0.31, 0.55]   # Bottom right CAD
    ], dtype=np.float32)

    # 3. Calculate and return the 3x3 transformation matrix
    H_matrix = cv.getPerspectiveTransform(src_pts, dst_pts)
    return H_matrix

def px_to_xy_homography(px_x, px_y, H_matrix):
    """
    Transforms a single pixel coordinate into real-world distance and angle.
    """
    # OpenCV perspectiveTransform expects a specific 3D array shape: (1, 1, 2)
    pt = np.array([[[px_x, px_y]]], dtype=np.float32)
    
    # Apply the homography matrix
    transformed_pt = cv.perspectiveTransform(pt, H_matrix)
    
    # Extract the real-world X and Y in meters
    real_x = transformed_pt[0][0][0]
    real_y = transformed_pt[0][0][1]
    
    # Calculate straight-line distance to the target (Hypotenuse)
    target_distance = np.sqrt(real_x**2 + real_y**2)
    
    # Calculate the angle to turn (in radians)
    # Positive angle = left (counterclockwise), matching the robot's turn convention.
    # real_x is positive-right, so negate it for positive-left heading.
    target_angle = np.arctan2(-real_x, real_y)
    
    return target_distance, target_angle, real_x, real_y

def find_and_catch():
    """Main mission state machine with two-phase approach"""
    state = 0
    arm_reach = 0.26 # middle of the cup is 26cm from the center of the robot
    approach_margin = 0.20 # stop 20cm before arm reach on first pass
    real_x = 0.0
    real_y = 0.0
    target_distance = 0.0
    target_angle = 0.0
    drive_dist = 0.0
    H_matrix = None 
    
    print("% Starting Golf Mission: Find and Catch the Red Ball!")
    service.send("robobot/cmd/T0", "leds 16 0 0 100") # Blue LED: Searching
    service.send("robobot/cmd/T0", "servo 1 -900 300") # Ensure arm is UP
    
    while not service.stop:
        if state == 0: 
            # STATE 0: Look for the ball
            ok, img, imgTime = cam.getImage()
            if ok:
                h, w, _ = img.shape
                
                if H_matrix is None:
                    H_matrix = setup_homography(w, h)
                    print(f"% Homography matrix initialized for {w}x{h} camera.")

                found, px_x, px_y, radius, mask = detect_red_ball(img)
                
                if found:
                    service.send("robobot/cmd/T0", "leds 16 0 100 0") # Green LED: Found
                    service.send("robobot/cmd/ti", "rc 0.0 0.0") # Stop spinning
                    
                    target_distance, target_angle, real_x, real_y = px_to_xy_homography(px_x, px_y, H_matrix)
                    drive_dist = target_distance - arm_reach - approach_margin
                    
                    print(f"% Ball found at Px({px_x}, {px_y}), radius={radius:.1f}")
                    print(f"%   World position: x={real_x:.3f} m, y={real_y:.3f} m")
                    print(f"%   Distance={target_distance:.3f} m, Angle={np.degrees(target_angle):.1f} deg")
                    print(f"%   Phase 1 drive: {drive_dist:.3f} m (dist - arm - margin)")
                    
                    pose.tripBreset() 
                    state = 1
                else:
                    service.send("robobot/cmd/ti", "rc 0.0 0.4") # Spin to search
                    
        elif state == 1:
            # STATE 1: Turn towards the ball
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
            # STATE 2: Drive towards the ball (phase 1 — stop short)
            if drive_dist <= 0:
                print(f"% Already close enough for phase 2 (drive_dist={drive_dist:.3f} m)")
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                t.sleep(0.3)
                state = 10
            else:
                service.send("robobot/cmd/ti", "rc 0.2 0.0")
                if pose.tripB >= drive_dist:
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                    print(f"% Phase 1 done. Driven {pose.tripB:.3f} m (target {drive_dist:.3f} m)")
                    t.sleep(0.3)
                    state = 10

        elif state == 10:
            # STATE 10: Re-detect the ball from closer range
            ok, img, imgTime = cam.getImage()
            if ok:
                found, px_x, px_y, radius, mask = detect_red_ball(img)
                if found:
                    target_distance, target_angle, real_x, real_y = px_to_xy_homography(px_x, px_y, H_matrix)
                    drive_dist = target_distance - arm_reach
                    
                    print(f"% Phase 2: Ball at Px({px_x}, {px_y}), radius={radius:.1f}")
                    print(f"%   World position: x={real_x:.3f} m, y={real_y:.3f} m")
                    print(f"%   Distance={target_distance:.3f} m, Angle={np.degrees(target_angle):.1f} deg")
                    print(f"%   Phase 2 drive: {drive_dist:.3f} m (dist - arm)")
                    
                    pose.tripBreset()
                    state = 11
                else:
                    print("% Phase 2: Ball not found, catching blind")
                    drive_dist = approach_margin
                    pose.tripBreset()
                    state = 12

        elif state == 11:
            # STATE 11: Fine-adjust heading towards the ball
            if abs(target_angle) < 0.05:
                # Close enough, skip turning
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
                    print(f"% Phase 2 turn: {np.degrees(pose.tripBh):.1f} deg")
                    pose.tripBreset()
                    state = 12

        elif state == 12:
            # STATE 12: Drive the final stretch to arm reach
            if drive_dist <= 0:
                print(f"% Already at arm reach")
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                state = 3
            else:
                service.send("robobot/cmd/ti", "rc 0.15 0.0") # Slower for precision
                if pose.tripB >= drive_dist:
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                    print(f"% Phase 2 done. Driven {pose.tripB:.3f} m (target {drive_dist:.3f} m)")
                    state = 3
                    
        elif state == 3:
            # STATE 3: Catch!
            print("% Catching the ball!")
            service.send("robobot/cmd/T0", "servo 1 100 400") 
            t.sleep(1.5) 
            state = 99
            
        elif state == 99:
            # STATE 99: Finished
            print("% Mission Accomplished.")
            service.send("robobot/cmd/T0", "leds 16 100 0 100") # Purple LED: Done
            break
            
        t.sleep(0.05)


def find_and_print():
    """
    Takes a single picture, saves all debug masks, and prints coordinates.
    """
    print("% Starting Golf Test: Find, Print, and Debug!")
    os.makedirs("golf_test_results", exist_ok=True)
    
    while not service.stop:
        ok, img, imgTime = cam.getImage()
        
        if ok:
            h, w, _ = img.shape
            H_matrix = setup_homography(w, h)
            
            # Process the image and get our dictionary of debug images
            found, px_x, px_y, radius, debug_images = detect_red_ball(img)
            
            timestamp = imgTime.strftime('%Y%m%d_%H%M%S')
            
            # Save every image in the dictionary
            print("% Saving debug images...")
            for name, image_data in debug_images.items():
                filename = f"golf_test_results/{timestamp}_{name}.jpg"
                cv.imwrite(filename, image_data)
                print(f"  -> Saved {name}")
            
            if found:
                target_dist, target_angle, real_x, real_y = px_to_xy_homography(px_x, px_y, H_matrix)
                
                print("\n" + "="*40)
                print("           BALL FOUND!")
                print("="*40)
                print(f"Pixel Coordinates : X={px_x}, Y={px_y}, Radius={radius:.1f}")
                print(f"World Position    : x={real_x:.3f}m (+ right), y={real_y:.3f}m (forward)")
                print(f"Robot Targets     : Dist={target_dist:.3f}m, Angle={np.degrees(target_angle):.1f}deg")
                print("="*40 + "\n")
            else:
                print("\n" + "="*40)
                print("       NO BALL FOUND IN IMAGE")
                print("="*40 + "\n")
            
            service.stop = True
            break
        
        t.sleep(0.1)