import time
import math
import numpy as np
from biscaROS.core.exceptions import MissionAborted
from biscaROS.core.map_data import get_waypoint

_node = None

def init(node_instance):
    global _node
    _node = node_instance

def check_abort():
    if _node.abort_flag:
        raise MissionAborted("Mission aborted by user.")

def straight(distance=1.0, speed=0.2, timeout=20.0, stop_at_end=True):
    print(f"[Drive] Straight {distance}m at {speed}m/s")
    start_x = _node.state.get('robot_x', 0.0)
    start_y = _node.state.get('robot_y', 0.0)
    start_time = time.time()

    _node.publish("robobot/cmd/ti", f"rc {speed} 0.0")

    while True:
        check_abort()
        curr_x = _node.state.get('robot_x', 0.0)
        curr_y = _node.state.get('robot_y', 0.0)
        travelled = math.hypot(curr_x - start_x, curr_y - start_y)

        if travelled >= distance or (time.time() - start_time) > timeout:
            break
        time.sleep(0.02)

    if stop_at_end:
        _node.publish("robobot/cmd/ti", "rc 0.0 0.0")

def turn(angle_deg, speed=0.3, timeout=15.0):
    print(f"[Drive] Turning {angle_deg}°")
    start_yaw = _node.state.get('robot_yaw', 0.0)
    target_yaw = start_yaw + math.radians(angle_deg)
    # Normalize to [-pi, pi]
    target_yaw = (target_yaw + math.pi) % (2 * math.pi) - math.pi 
    
    turn_rate = speed if angle_deg > 0 else -speed
    _node.publish("robobot/cmd/ti", f"rc 0.0 {turn_rate:.3f}")
    
    start_time = time.time()
    while True:
        check_abort()
        curr_yaw = _node.state.get('robot_yaw', 0.0)
        diff = abs((target_yaw - curr_yaw + math.pi) % (2 * math.pi) - math.pi)
        
        if diff < 0.05 or (time.time() - start_time) > timeout:
            break
        time.sleep(0.02)
        
    _node.publish("robobot/cmd/ti", "rc 0.0 0.0")

def curved(heading_deg, nominal_speed=0.2, turn_rate=0.01, timeout=20.0, tolerance_deg=2.0):
    print(f"[Drive] Curved to {heading_deg}°")
    target_rad = math.radians(heading_deg)
    tolerance = math.radians(tolerance_deg)
    
    # Determine turn direction based on shortest path
    curr_yaw = _node.state.get('robot_yaw', 0.0)
    diff = (target_rad - curr_yaw + math.pi) % (2 * math.pi) - math.pi
    sign = 1 if diff > 0 else -1
    
    _node.publish("robobot/cmd/ti", f"rc {nominal_speed} {sign * turn_rate:.3f}")
    
    start_time = time.time()
    while True:
        check_abort()
        curr_yaw = _node.state.get('robot_yaw', 0.0)
        diff = abs((target_rad - curr_yaw + math.pi) % (2 * math.pi) - math.pi)
        
        if diff <= tolerance or (time.time() - start_time) > timeout:
            break
        time.sleep(0.02)
        
    _node.publish("robobot/cmd/ti", "rc 0.0 0.0")

def to_xy(target_x, target_y, speed=0.2, timeout=30.0, dist_tol=0.05):
    print(f"[Drive] Driving to X:{target_x}, Y:{target_y}")
    start_time = time.time()
    
    while True:
        check_abort()
        curr_x = _node.state.get('robot_x', 0.0)
        curr_y = _node.state.get('robot_y', 0.0)
        curr_h = _node.state.get('robot_yaw', 0.0)
        
        dx = target_x - curr_x
        dy = target_y - curr_y
        distance = math.hypot(dx, dy)
        
        if distance <= dist_tol or (time.time() - start_time) > timeout:
            break
            
        target_heading = math.atan2(dy, dx)
        heading_error = (target_heading - curr_h + math.pi) % (2 * math.pi) - math.pi
        
        # Simple P-Controller for steering towards point
        turn_speed = max(min(0.8 * heading_error, 1.0), -1.0)
        _node.publish("robobot/cmd/ti", f"rc {speed:.3f} {turn_speed:.3f}")
        time.sleep(0.02)
        
    _node.publish("robobot/cmd/ti", "rc 0.0 0.0")

def to_waypoint(waypoint_name, speed=0.2, timeout=30.0, dist_tol=0.05):
    wp = get_waypoint(waypoint_name)
    print(f"[Drive] Navigating to waypoint '{waypoint_name}'")
    to_xy(target_x=wp["x"], target_y=wp["y"], speed=speed, timeout=timeout, dist_tol=dist_tol)
    
    # Optional: Align to the waypoint's specific yaw after arrival
    if "yaw" in wp:
        target_deg = math.degrees(wp["yaw"])
        turn(angle_deg=(target_deg - math.degrees(_node.state.get('robot_yaw', 0))), speed=0.3)