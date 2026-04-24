import time
import math
from biscaROS.core.exceptions import MissionAborted

_node = None

def init(node_instance):
    global _node
    _node = node_instance

def check_abort():
    if _node.abort_flag:
        # Failsafe: Tell edge node to stop PID control
        _node.publish("biscaROS/control/edge", "disable") 
        raise MissionAborted("Mission aborted by user.")

def follow(side="left", speed=0.2, max_speed=0.25, brake_speed=-0.05, timeout=30.0):
    print(f"[Line] Following {side} edge")
    start_time = time.time()
    
    # 1. Approach: Drive straight until line found
    _node.publish("robobot/cmd/ti", f"rc {speed:.3f} 0.0")
    while True:
        check_abort()
        if _node.state.get('edge_valid', False):
            break
        if (time.time() - start_time) > timeout:
            _node.publish("robobot/cmd/ti", "rc 0.0 0.0")
            print("[Line] Timeout searching for line")
            return
        time.sleep(0.02)

    # 2. Follow: Delegate PID to the edge node
    _node.publish("biscaROS/control/edge", f"enable:{side}:{speed}:{max_speed}:{brake_speed}")
    
    # Block until line is lost or timeout
    while True:
        check_abort()
        if not _node.state.get('edge_valid', True):
            print("[Line] Line lost.")
            break
        if (time.time() - start_time) > timeout:
            print("[Line] Timeout reached.")
            break
        time.sleep(0.02)
        
    _node.publish("biscaROS/control/edge", "disable")
    _node.publish("robobot/cmd/ti", "rc 0.0 0.0")