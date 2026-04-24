# biscaROS/missions/complete_mission.py
from biscaROS.tasks import drive, vision, kalman, line

def run():
    print("--- Starting Complete Mission ---")
    
    # ----- Start -----
    # Assuming 'start' is X:0, Y:0, Yaw:0
    kalman.set_pose(0.0, 0.0, 0.0) 
    
    vision.follow_line(timeout=10.0) # Drives until 'end_of_line' condition
    drive.straight(distance=0.5)
    drive.curved(heading_deg=-15)
    
    vision.roundabout(speed=0.4, percentage_of_white=35) 
    
    drive.straight(distance=0.4)
    vision.follow_line(timeout=3.0) 

    # ----- Marker -----
    # Assuming 'pose_A' coordinates
    kalman.set_pose(x=1.5, y=2.0, yaw=0.0) 
    
    # Assuming 'time_marker_A' coordinates
    drive.to_xy(target_x=2.5, target_y=2.0)
    drive.turn(angle_deg=90) # Adjust to final yaw

    # ----- Eight/police -----
    kalman.set_pose(x=2.5, y=2.0, yaw=1.57)
    
    vision.wait_for_police()
    
    drive.straight(distance=0.5)
    drive.turn(angle_deg=-70)
    drive.straight(distance=0.5)

    # ----- Ramp -----
    print("[Mission] Executing Ramp")
    # Increase speed to climb the ramp
    drive.straight(distance=1.2, speed=0.4)

    # ----- Seesaw -----
    print("[Mission] Executing Seesaw")
    # Use hardware IR sensors for precise balancing over the seesaw
    line.follow(side="left", speed=0.15, timeout=15.0)

    print("--- Complete Mission Finished ---")