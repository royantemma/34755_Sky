import json
import time
from nodes.mqtt_base import MQTTNode
from missions.cube_catch import get_cube_catch_mission

class MissionNode(MQTTNode):
    def __init__(self):
        super().__init__("MissionNode")
        self.current_mission = None
        self.task_idx = 0
        self.last_known_cube = None
        self.latest_pose = None

    def on_connect(self, client, userdata, flags, rc):
        super().on_connect(client, userdata, flags, rc)
        self.client.subscribe("robobot/vision/aruco")
        self.client.subscribe("robobot/state/pose")
        self.client.subscribe("robobot/cmd/mission")
        # Ensure default states
        self.client.publish("robobot/cmd/ti", "rc 0.0 0.0")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        try:
            payload = msg.payload.decode('utf-8')
        except:
            return

        if topic == "robobot/vision/aruco":
            data = json.loads(payload)
            self.last_known_cube = data.get("cube_center")

        elif topic == "robobot/state/pose":
            self.latest_pose = json.loads(payload)

        elif topic == "robobot/cmd/mission":
            cmd = json.loads(payload)
            if cmd.get("command") == "start":
                mission_name = cmd.get("name")
                if mission_name == "cube_catch":
                    print(f"[{self.node_name}] Received command to start cube_catch mission!")
                    self.current_mission = get_cube_catch_mission()
                    self.task_idx = 0
            elif cmd.get("command") == "stop":
                print(f"[{self.node_name}] Mission aborted via MQTT!")
                self.current_mission = None
                self.client.publish("robobot/cmd/ti", "rc 0.0 0.0")

    def run(self):
        print(f"[{self.node_name}] Ready and waiting for commands on 'robobot/cmd/mission'...")
        while self.running:
            if self.current_mission and self.task_idx < len(self.current_mission):
                active_task = self.current_mission[self.task_idx]
                
                if not active_task.started:
                    active_task.start(self)
                
                # Execute logic hook
                active_task.update(self)
                
                if active_task.done:
                    print(f"[{self.node_name}] Task {active_task.name} completed.")
                    self.task_idx += 1
            elif self.current_mission and self.task_idx >= len(self.current_mission):
                print(f"[{self.node_name}] Mission fully completed.")
                self.current_mission = None
                self.client.publish("robobot/cmd/ti", "rc 0.0 0.0")
                
            time.sleep(0.05)

def start_mission():
    node = MissionNode()
    node.start()