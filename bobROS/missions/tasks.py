import time
import math
import numpy as np

class Task:
    def __init__(self, name):
        self.name = name
        self.started = False
        self.done = False

    def start(self, node):
        print(f"[{node.node_name}] Starting task: {self.name}")
        self.started = True

    def update(self, node):
        # Override to implement logic. Return True if done.
        pass

class TaskSetServo(Task):
    def __init__(self, servo_id, position, wait_time=0.5):
        super().__init__(f"SetServo_{servo_id}_{position}")
        self.servo_id = servo_id
        self.position = position
        self.wait_time = wait_time
        self.start_time = 0

    def start(self, node):
        super().start(node)
        # Send raw string command via MQTT
        node.publish("robobot/cmd/T0", f"servo {self.servo_id} {self.position} 300")
        self.start_time = time.time()

    def update(self, node):
        if time.time() - self.start_time >= self.wait_time:
            self.done = True

class TaskDrive(Task):
    def __init__(self, forward_vel, turn_vel, duration):
        super().__init__(f"Drive_{forward_vel}_{turn_vel}")
        self.forward = forward_vel
        self.turn = turn_vel
        self.duration = duration
        self.start_time = 0

    def start(self, node):
        super().start(node)
        self.start_time = time.time()
    
    def update(self, node):
        node.publish("robobot/cmd/ti", f"rc {self.forward} {self.turn}")
        if time.time() - self.start_time >= self.duration:
            node.publish("robobot/cmd/ti", "rc 0.0 0.0")
            self.done = True

class TaskSpinUntilCameraSees(Task):
    def __init__(self, marker_type="cube_center"):
        super().__init__("SpinUntilDetected")
        self.marker_type = marker_type

    def start(self, node):
        super().start(node)

    def update(self, node):
        # Check node's latest vision state
        if self.marker_type == "cube_center" and node.last_known_cube:
            # We see it! Stop spinning.
            node.publish("robobot/cmd/ti", "rc 0.0 0.0")
            print(f"[{node.node_name}] Found the cube!")
            node.publish("robobot/cmd/T0", "leds 16 0 100 0") # Green LED
            self.done = True
        else:
            # Keep spinning
            node.publish("robobot/cmd/ti", "rc 0.0 0.5")

class TaskApproachCube(Task):
    def __init__(self, stop_distance=0.29):
        super().__init__("ApproachCube")
        self.stop_distance = stop_distance

    def start(self, node):
        super().start(node)
    
    def update(self, node):
        cube = node.last_known_cube
        if not cube:
            print(f"[{node.node_name}] Lost cube! Pausing approach...")
            node.publish("robobot/cmd/ti", "rc 0.0 0.0")
            return
            
        rx = cube["x"] / 1000.0
        ry = cube["y"] / 1000.0
        
        target_distance = np.sqrt(rx**2 + ry**2)
        target_angle = np.arctan2(-rx, ry)
        
        # Turn correction
        turn_vel = 0.0
        if abs(target_angle) > 0.05:
            turn_vel = 0.3 if target_angle > 0 else -0.3
            
        # Stopping condition
        drive_dist = target_distance - self.stop_distance
        
        if drive_dist <= 0:
            print(f"[{node.node_name}] Reached cube at distance {target_distance}")
            node.publish("robobot/cmd/ti", "rc 0.0 0.0")
            self.done = True
        else:
            # Approach
            fwd_vel = 0.2 if abs(target_angle) < 0.1 else 0.0
            node.publish("robobot/cmd/ti", f"rc {fwd_vel} {turn_vel}")
