# biscaROS/core/map_data.py

WAYPOINTS = {
    "start": {"x": 0.0, "y": 0.0, "yaw": 0.0},
    "pose_A": {"x": 1.5, "y": 2.0, "yaw": 0.0},
    "time_marker_A": {"x": 2.5, "y": 2.0, "yaw": 1.57}
}

def get_waypoint(name):
    if name not in WAYPOINTS:
        raise ValueError(f"Waypoint '{name}' not found in map_data.py")
    return WAYPOINTS[name]