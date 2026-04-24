import time
from biscaROS.core.exceptions import MissionAborted

_node = None

def init(node_instance):
    global _node
    _node = node_instance

def set_position(position, speed=300):
    """Generic servo position (e.g. -900 to 150)"""
    _node.publish("robobot/cmd/T0", f"servo 1 {position} {speed}")
    time.sleep(0.5) # Allow physical movement

def up():
    print("[Servo] Arm Up")
    set_position(-900, 300)

def down():
    print("[Servo] Arm Down")
    set_position(150, 400)
    
def disable():
    print("[Servo] Disabling hold")
    _node.publish("robobot/cmd/T0", "servo 1 disable")