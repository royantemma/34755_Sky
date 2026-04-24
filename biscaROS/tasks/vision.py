import time
from biscaROS.core.exceptions import MissionAborted

_node = None

def init(node_instance):
    global _node
    _node = node_instance

def check_abort():
    if _node.abort_flag:
        _node.publish("biscaROS/control/vision", "stop")
        raise MissionAborted("Mission aborted by user.")

def wait_for_police(timeout=20.0):
    print("[Vision] Waiting for police clear")
    _node.publish("biscaROS/control/vision", "start_police_wait")
    
    start_time = time.time()
    while True:
        check_abort()
        # Assume vision node publishes "clear" when police move
        if _node.state.get('police_status') == 'clear':
            break
        if (time.time() - start_time) > timeout:
            break
        time.sleep(0.05)
        
    _node.publish("biscaROS/control/vision", "stop")

def follow_line(speed=0.4, stop_speed=0.0, timeout=30.0):
    print("[Vision] Following line")
    _node.publish("biscaROS/control/vision", f"start_line_follow:{speed}")
    
    start_time = time.time()
    while True:
        check_abort()
        # Assume vision node publishes "end_of_line" when it loses track
        if _node.state.get('vision_line_status') == 'end_of_line':
            break
        if (time.time() - start_time) > timeout:
            break
        time.sleep(0.05)
        
    _node.publish("biscaROS/control/vision", "stop")
    if stop_speed <= 0:
        _node.publish("robobot/cmd/ti", "rc 0.0 0.0")