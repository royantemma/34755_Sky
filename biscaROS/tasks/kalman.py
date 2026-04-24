import json
from biscaROS.core.exceptions import MissionAborted
from biscaROS.core.map_data import get_waypoint

_node = None

def init(node_instance):
    global _node
    _node = node_instance

def set_pose(waypoint_name=None, x=None, y=None, yaw=0.0):
    if waypoint_name:
        wp = get_waypoint(waypoint_name)
        x, y, yaw = wp["x"], wp["y"], wp.get("yaw", 0.0)
        
    print(f"[Kalman] Resetting pose to X:{x}, Y:{y}, Yaw:{yaw}")
    payload = json.dumps({"x": x, "y": y, "yaw": yaw})
    _node.publish("biscaROS/navigation/correct", payload)