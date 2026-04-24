"""
The mission node is responsible for executing the mission files, when called via MQTT. It receives the name of the mission (or task), find it in the mission folder (or task folder), and executes the corresponding Python file.
"""

# biscaROS/nodes/mission_node.py
import threading
import time
import json
import importlib
from biscaROS.core.base_node import BaseNode

# Import task modules for dependency injection
# Note: You will create these in the next step
from biscaROS.tasks import drive, vision, kalman, servo

class MissionNode(BaseNode):
    def __init__(self):
        super().__init__(node_name="mission_orchestrator")
        
        # Central state dictionary updated by MQTT callbacks.
        # Task modules read from this dictionary (e.g., _node.state['tripB'])
        self.state = {
            'tripB': 0.0,
            'tripBh': 0.0,
            'robot_x': 0.0,
            'robot_y': 0.0,
            'robot_yaw': 0.0,
            'ir_distance': 99.0
        }
        
        self.abort_flag = False
        self._mission_thread = None
        
        # --- Option 1: Inject self into all task modules ---
        drive.init(self)
        vision.init(self)
        kalman.init(self)
        servo.init(self)
        
        # Subscribe to command topics and required telemetry
        self.client.subscribe("biscaROS/mission/cmd")
        self.client.subscribe("biscaROS/navigation/pose")
        self.client.subscribe("biscaROS/navigation/angle")
        self.client.subscribe("biscaROS/sensors/ir")

    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode('utf-8')
        
        if topic == "biscaROS/mission/cmd":
            if payload == "abort":
                self.abort_mission()
            elif payload.startswith("start:"):
                mission_name = payload.split(":")[1]
                self.start_mission(mission_name)
                
        # --- State Updates ---
        elif topic == "biscaROS/navigation/pose":
            try:
                coords = payload.split()
                self.state['robot_x'] = float(coords[0])
                self.state['robot_y'] = float(coords[1])
            except (ValueError, IndexError):
                pass
                
        elif topic == "biscaROS/navigation/angle":
            try:
                angles = payload.split()
                self.state['robot_yaw'] = float(angles[2])
            except (ValueError, IndexError):
                pass
                
        elif topic == "biscaROS/sensors/ir":
            try:
                # Assuming ir_node publishes "ir1,ir2"
                self.state['ir_distance'] = float(payload.split(',')[0])
            except (ValueError, IndexError):
                pass

    def start_mission(self, mission_name):
        if self._mission_thread and self._mission_thread.is_alive():
            print("[MissionNode] Rejected: A mission is already running.")
            return

        self.abort_flag = False
        
        try:
            # Dynamically import the requested mission script from the missions folder
            mission_module = importlib.import_module(f"biscaROS.missions.{mission_name}")
            target_func = mission_module.run
            
            # Start the execution wrapper in a background thread
            self._mission_thread = threading.Thread(target=self._run_wrapper, args=(target_func,))
            self._mission_thread.daemon = True
            self._mission_thread.start()
            
            print(f"[MissionNode] Started mission: {mission_name}")
            
        except ModuleNotFoundError:
            print(f"[MissionNode] Error: Mission '{mission_name}' not found.")
        except AttributeError:
            print(f"[MissionNode] Error: Mission '{mission_name}' lacks a run() function.")

    def abort_mission(self):
        print("[MissionNode] ABORT signal received.")
        self.abort_flag = True
        # Failsafe hardware stop. The task thread will throw an exception shortly after.
        self.publish("robobot/cmd/ti", "rc 0.0 0.0")

    def _run_wrapper(self, mission_func):
        """Executes the mission and catches the abort exception."""
        try:
            # Execute the procedural script
            mission_func()
            print("[MissionNode] Mission completed successfully.")
            
        except Exception as e:
            # If the exception is our abort exception, handle it cleanly
            if type(e).__name__ == "MissionAborted":
                print("[MissionNode] Mission aborted safely.")
            else:
                print(f"[MissionNode] Mission crashed with error: {e}")
                
            # Guarantee the robot stops regardless of how the script exited
            self.publish("robobot/cmd/ti", "rc 0.0 0.0")

if __name__ == "__main__":
    node = MissionNode()
    node.start()
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        node.abort_mission()
        node.stop()