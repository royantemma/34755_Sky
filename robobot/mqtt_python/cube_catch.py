import time as t
import numpy as np
import cv2 as cv
import os

# Import framework modules
from uservice import service
from sfuse import iwo as pose
import aruco

def find_and_catch():
    """Main mission state machine with two-phase approach for ArUco Cube"""
    state = 0
    arm_reach = 0.29
    approach_margin = 0.40 # stop 40cm before arm reach on first pass
    
    target_distance = 0.0
    target_angle = 0.0
    drive_dist = 0.0
    
    client = aruco.get_client()
    client.set_enabled(True)

    print("% Starting Cube Catch Mission (Shared Aruco)!")
    service.send("robobot/cmd/T0", "leds 16 0 0 100") # Blue LED: Searching
    service.send("robobot/cmd/T0","servo 1 -930 400") # Ensure arm is UP
    
    t.sleep(1.0)
    
    while not service.stop:
        if state == 0: 
            # STATE 0: Look for the cube
            data = client.get_data()
            if data and data.get("cube_center"):
                service.send("robobot/cmd/T0", "leds 16 0 100 0") # Green LED: Found
                service.send("robobot/cmd/ti", "rc 0.0 0.0") # Stop spinning
                
                # Fetch Coordinates directly from streaming server
                rx = data["cube_center"]["x"] / 1000.0
                ry = data["cube_center"]["y"] / 1000.0
                rz = data["cube_center"]["z"] / 1000.0
                
                target_distance = np.sqrt(rx**2 + ry**2)
                target_angle = np.arctan2(-rx, ry)
                
                drive_dist = target_distance - arm_reach - approach_margin
                
                print(f"% Cube found! Center at: x={rx:.3f} m, y={ry:.3f} m (Z={rz:.3f}m)")
                print(f"%   Distance={target_distance:.3f} m, Angle={np.degrees(target_angle):.1f} deg")
                print(f"%   Phase 1 drive: {drive_dist:.3f} m (dist - arm - margin)")
                
                pose.tripBreset() 
                state = 1
            else:
                # Spin to search
                service.send("robobot/cmd/ti", "rc 0.0 0.1")
                t.sleep(0.1)
                
        elif state == 1:
            # STATE 1: Turn towards the cube
            if target_angle > 0.05: # About 3 degrees tolerance
                service.send("robobot/cmd/ti", "rc 0.0 0.5")
            else:
                service.send("robobot/cmd/ti", "rc 0.0 -0.5")
                
            if abs(pose.tripBh) >= abs(target_angle):
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print(f"% Turned {np.degrees(pose.tripBh):.1f} deg (target {np.degrees(target_angle):.1f} deg)")
                pose.tripBreset()
                state = 2
            t.sleep(0.05)

        elif state == 2:
            # STATE 2: Drive towards the cube (phase 1)
            if drive_dist <= 0:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                t.sleep(0.3)
                state = 10
            else:
                service.send("robobot/cmd/ti", "rc 0.4 0.0")
                if pose.tripB >= drive_dist:
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                    t.sleep(0.5)
                    state = 10
            t.sleep(0.05)

        elif state == 10:
            # STATE 10: Re-detect the cube closely
            data = client.get_data()
            if data and data.get("cube_center"):
                rx = data["cube_center"]["x"] / 1000.0
                ry = data["cube_center"]["y"] / 1000.0
                
                target_distance = np.sqrt(rx**2 + ry**2)
                target_angle = np.arctan2(-rx, ry)
                drive_dist = target_distance - arm_reach
                
                print(f"% Phase 2: Cube at x={rx:.3f} m, y={ry:.3f} m")
                print(f"%   Distance={target_distance:.3f} m, Angle={np.degrees(target_angle):.1f} deg")
                print(f"%   Phase 2 drive: {drive_dist:.3f} m (dist - arm)")
                
                pose.tripBreset()
                state = 11
            else:
                print("% Phase 2: Cube not tracked anymore, catching blind!")
                drive_dist = approach_margin
                pose.tripBreset()
                state = 12
            t.sleep(0.1)

        elif state == 11:
            # STATE 11: Fine-adjust heading
            if abs(target_angle) < 0.02: # About 1 degree tolerance
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                pose.tripBreset()
                state = 12
            else:
                if target_angle > 0:
                    service.send("robobot/cmd/ti", "rc 0.0 0.2")
                else:
                    service.send("robobot/cmd/ti", "rc 0.0 -0.2")
                    
                if abs(pose.tripBh) >= abs(target_angle) - 0.01: # Stop early to avoid overshoot
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                    print(f"% Turned {np.degrees(pose.tripBh):.1f} deg (target {np.degrees(target_angle):.1f} deg)")
                    pose.tripBreset()
                    state = 12
            t.sleep(0.05)
            
        elif state == 12:
            # Drive the last segment
            if drive_dist <= 0:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                state = 13
            else:
                service.send("robobot/cmd/ti", "rc 0.1 0.0")
                if pose.tripB >= drive_dist:
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                    state = 13
            t.sleep(0.05)
            
        elif state == 13: # gross catch
            print("% Lowering arm to catch cube...")
            service.send("robobot/cmd/T0","servo 1 150 400") # (servo down)
            t.sleep(0.5)
            service.send("robobot/cmd/T0","servo 1 10000 0") # (servo off)
            state = 14

        elif state == 14: # fine catch
            # jiggle left and right to ensure cube is in arm
            service.send("robobot/cmd/ti", "rc 0.0 0.4")
            t.sleep(1)
            service.send("robobot/cmd/ti", "rc 0.0 -0.4")
            t.sleep(2)
            service.send("robobot/cmd/ti", "rc 0.0 0.0")

            print("% Cube caught! (hopefully...)")
            t.sleep(0.5)

            # Disabling servo
            service.send("robobot/cmd/T0","servo 1 10000 0") # (servo off)
            
            client.set_enabled(False)
            service.stop = True
            break
            
