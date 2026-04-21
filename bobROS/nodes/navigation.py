import time
import json
import sys
import os

from lib.estimation.kalman import iwo
from nodes.mqtt_base import MQTTNode

class NavigationNode(MQTTNode):
    def __init__(self):
        super().__init__("NavigationNode")
        iwo.setup() # Initialize the Kalman filter matrices

    def on_connect(self, client, userdata, flags, rc):
        super().on_connect(client, userdata, flags, rc)
        self.client.subscribe("robobot/drive/imu")
        self.client.subscribe("robobot/drive/motor")
        self.client.subscribe("robobot/drive/pose")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode('utf-8')
        
        # Strip subtopic logic that sfuse.decode normally expects
        subtopic = topic.replace("robobot/drive/", "")
        iwo.decode(subtopic, payload)

    def run(self):
        print(f"[{self.node_name}] Kalman Filter (Navigation) running...")
        while self.running:
            # Periodically publish the current estimate
            pose_data = {
                "x": iwo.pose[0],
                "y": iwo.pose[1],
                "h": iwo.pose[2],
                "tripB": iwo.tripB,
                "tripBh": iwo.tripBh
            }
            self.client.publish("robobot/state/pose", json.dumps(pose_data))
            time.sleep(0.01) # 100Hz tick rate

def start_navigation():
    node = NavigationNode()
    node.start()