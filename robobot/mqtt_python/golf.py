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
        
        if radius > 10: 
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
    src_pts = np.array([
        [0, img_height / 2.0],           # Middle left pixel
        [img_width, img_height / 2.0],   # Middle right pixel
        [0, img_height],                 # Bottom left pixel
        [img_width, img_height]          # Bottom right pixel
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
    dst_pts = np.array([ # For z=21mm (middle of the ball)
        [-0.510, 0.925],   # Middle left CAD
        [0.510, 0.925],    # Middle right CAD
        [-0.157, 0.228], # Bottom left CAD
        [0.157, 0.228]   # Bottom right CAD
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
    # Assuming Y is forward and X is horizontal
    target_angle = np.arctan2(real_x, real_y)
    
    return target_distance, target_angle

def find_and_catch():
    """Main mission state machine"""
    state = 0
    arm_reach = 0.26 # middle of the cup is 26cm from the center of the robot
    target_distance = 0.0
    target_angle = 0.0
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
                
                # Setup Homography Matrix on the very first frame
                if H_matrix is None:
                    H_matrix = setup_homography(w, h)
                    print(f"% Homography matrix initialized for {w}x{h} camera.")

                found, px_x, px_y, radius, mask = detect_red_ball(img)
                
                if found:
                    print(f"% Ball found at Px({px_x}, {px_y})!")
                    service.send("robobot/cmd/T0", "leds 16 0 100 0") # Green LED: Found
                    service.send("robobot/cmd/ti", "rc 0.0 0.0") # Stop spinning
                    
                    # Calculate true trajectory using the matrix
                    target_distance, target_angle = px_to_xy_homography(px_x, px_y, H_matrix)
                    
                    pose.tripBreset() 
                    state = 1
                else:
                    # Spin slowly to search for the ball
                    service.send("robobot/cmd/ti", "rc 0.0 0.4") 
                    
        elif state == 1:
            # STATE 1: Turn towards the ball
            if target_angle > 0:
                service.send("robobot/cmd/ti", "rc 0.0 0.5") # Turn left
            else:
                service.send("robobot/cmd/ti", "rc 0.0 -0.5") # Turn right
                
            # Check if we have turned enough using odometry (tripBh is heading)
            if abs(pose.tripBh) >= abs(target_angle):
                service.send("robobot/cmd/ti", "rc 0.0 0.0") # Stop turning
                pose.tripBreset() # Reset distance counter
                state = 2
                print(f"% Turned {pose.tripBh:.2f} rad. Now driving to target.")

        elif state == 2:
            # STATE 2: Drive towards the ball
            drive_dist = target_distance - arm_reach
            
            if drive_dist <= 0:
                # Already in reach!
                state = 3
            else:
                service.send("robobot/cmd/ti", "rc 0.2 0.0") # Drive forward at 0.2 m/s
                
                # Check if we have driven far enough (tripB is distance)
                if pose.tripB >= drive_dist:
                    service.send("robobot/cmd/ti", "rc 0.0 0.0") # Stop
                    state = 3
                    print(f"% Arrived at ball. Driven {pose.tripB:.2f} m.")
                    
        elif state == 3:
            # STATE 3: Catch!
            print("% Catching the ball!")
            # Move servo to lower position
            service.send("robobot/cmd/T0", "servo 1 100 400") 
            
            # Wait a moment for the servo to physically move
            t.sleep(1.5) 
            state = 99
            
        elif state == 99:
            # STATE 99: Finished
            print("% Mission Accomplished.")
            service.send("robobot/cmd/T0", "leds 16 100 0 100") # Purple LED: Done
            break
            
        # Do not loop too fast, give the CPU a break
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
                target_dist, target_angle = px_to_xy_homography(px_x, px_y, H_matrix)
                
                print("\n" + "="*40)
                print("           BALL FOUND!")
                print("="*40)
                print(f"Pixel Coordinates : X={px_x}, Y={px_y}, Radius={radius:.1f}")
                print(f"Robot Targets     : Dist={target_dist:.3f}m, Angle={target_angle:.3f}rad")
                print("="*40 + "\n")
            else:
                print("\n" + "="*40)
                print("       NO BALL FOUND IN IMAGE")
                print("="*40 + "\n")
            
            service.stop = True
            break
        
        t.sleep(0.1)