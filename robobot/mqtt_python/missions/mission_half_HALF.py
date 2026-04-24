"""
missions/mission_gates.py
---------------------
Test Route: Start -> drive 3.5m -> turn left 90 -> line follow to (5.41, -2.00) 
            -> drive to extra time pt 1 -> return to xy -> line follow ramp
"""

TOTAL_TIME       = 120
GOAL_TIME_BUFFER = 20

TASKS = [

    # 1. Drive forward 3.5 meters
    {
        "type":     "drive_straight",
        "speed":    0.3,
        "distance": 3.5,
        "timeout":  40,
    },

    # 2. Turn left 90 degrees
    # Positive angles are counter-clockwise (Left)
    {
        "type":      "turn",
        "angle_deg": 70, 
        "speed":     0.4,
        "timeout":   15,
    },

    # 3. Approach line & Follow to (5.41, -2.33)
    {
        "type":     "line_follow_until_xy",
        "side":     "right",
        "speed":    0.3,
        "target_x": 5.41,
        "target_y": -2.33,
        "distance_tolerance": 0.1,
        "timeout":  40
    },

    # 4. Adjust to zero heading
    {
        "type":          "turn_to_heading",
        "heading_deg":   0, 
        "speed":         0.4,
        "tolerance_deg": 1.5,
        "timeout":       10
    },

    # 5. Reverse to Extra Time Point 1
    {
        "type":        "drive_straight_heading",
        "speed":       -0.3,  # Negative = Reverse
        "distance":    2.10,  # 5.41 - 3.31
        "heading_deg": 0,     # Maintain 0 degrees while backing up
        "timeout":     15
    },

    # 6. Drive Forward to return to the exit point
    {
        "type":        "drive_straight_heading", 
        "speed":       0.3,   # Positive = Forward
        "distance":    2.10, 
        "heading_deg": 0,     # Maintain 0 degrees while driving forward
        "timeout":     15
    },

    # 7. Catch the line and continue up the ramp
    {
        "type":     "line_follow",
        "side":     "right",
        "speed":    0.4,
        "timeout":  30
    },

    # golf balls

    # 1. golf ball on seesaw

    # add left junction line follow task to get on seesaw

    # approach pos on seesaw

    {
        "type": "go_to_xy", # coord on seesaw
        "target_x": 1.5, # Example coords just outside the ball's location
        "target_y": 2.0,
        "speed": 0.4
    },

    # task to lower arm and trap golf ball
    {
        "type": "golf_visual_approach_and_trap",
        "timeout": 15,
        "forward_gain": 0.6,
        "turn_gain": 1.2
    },

    # add task to drive forward x amount

    # add task to update pos with luggage drop aruco

    # add task to drive to xy on before ramp

    # add task to drive line follow up ramp


    # navigate to approx pos to drop golf ball in hole
    {
        "type": "drive_to_xyyaw_IWO_fast",
        "target_x": 3.0, 
        "target_y": 4.5,
        "speed": 0.4
    },
    # release the ball into the hole
    {
        "type": "golf_release",
        "timeout": 3
    }

    # 2. golf ball on platform
    {
        "type": "drive_to_xyyaw_IWO_fast", # coord on platform (basically just turn around)
        "target_x": 1.5, 
        "target_y": 2.0,
        "speed": 0.4
    },

    {
        "type": "golf_visual_approach_and_trap",
        "timeout": 15,
        "forward_gain": 0.6,
        "turn_gain": 1.2
    },

    {
        "type": "go_to_xy",
        "target_x": 3.0, 
        "target_y": 4.5,
        "speed": 0.4
    },

    {
        "type": "golf_release",
        "timeout": 3
    }
]

img = cam.getImage()
found, px_x, px_y, radius, debug_images = detect_red_ball(img)

if found:
    target_dist, target_angle, real_x, real_y = px_to_xy_homography(px_x, px_y, H_matrix)
    
    Kp_turn = task.get("turn_gain", 1.0)
    Kp_fwd = task.get("forward_gain", 0.5)
    
    turn_rate = target_angle * Kp_turn
    
    # Slow down when closer to target
    if target_dist > 0.20: 
        forward_speed = 0.15 # Max approach speed
    else:
        # Scale speed down
        forward_speed = target_dist * Kp_fwd 
        
    # check if ball is within 4 cm and centered
    if target_dist < 0.04 and abs(target_angle) < 0.05:
        service.send("robobot/cmd/ti", "rc 0 0")
        
        # Update to Teensy servo command
        service.send("robobot/cmd/servo", "1 1000") 
        
        task_complete = True
    else:
        service.send("robobot/cmd/ti", f"rc {forward_speed:.3f} {turn_rate:.3f}")

else:
    # ball lost
    service.send("robobot/cmd/ti", "rc 0 0")
    
    
    # Optional: Implement a slow spin to search for the ball again
def run_golf_visual_approach_and_trap(self, task):
    print("--- Starting Golf Visual Approach ---")
    timeout = task.get("timeout", 15)
    Kp_fwd = task.get("forward_gain", 0.5)
    Kp_turn = task.get("turn_gain", 1.0)
    
    start_time = t.time()
    task_complete = False
    
    # Needs to loop until the ball is trapped or the timeout is reached
    while not task_complete and (t.time() - start_time) < timeout:
        # grab image and run detection from golf.py script
        img = cam.getImage()
        found, px_x, px_y, radius, _ = detect_red_ball(img)
        
        if found:

            target_dist, target_angle, _, _ = px_to_xy_homography(px_x, px_y, H_matrix)
            
            turn_rate = target_angle * Kp_turn
            
            # Slow down when getting close (20cm)
            if target_dist > 0.20:
                forward_speed = 0.15 
            else:
                forward_speed = target_dist * Kp_fwd 
                
            # check if ball is within 4 cm and centered
            if target_dist < 0.04 and abs(target_angle) < 0.05:
                
                service.send("robobot/cmd/ti", "rc 0 0")
                
                service.send("robobot/cmd/servo", "1 1000") # Update "1 1000" to your specific servo command
                t.sleep(0.5)
                
                print("--- Ball Trapped Successfully ---")
                task_complete = True
            else:
                # Drive towards the ball
                service.send("robobot/cmd/ti", f"rc {forward_speed:.3f} {turn_rate:.3f}")
                
        else:
            # ball lost. stop moving
            service.send("robobot/cmd/ti", "rc 0 0")
            
        t.sleep(0.05)
        
    if not task_complete:
        print("--- Task Failed: Golf Ball approach timed out ---")
        # Ensure robot is stopped if it times out
        service.send("robobot/cmd/ti", "rc 0 0")
