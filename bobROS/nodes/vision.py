import time
import json
import sys
import os
from threading import Condition
import io

# We use bobROS local imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from lib.vision.aruco import ArucoProcessor
from nodes.mqtt_base import MQTTNode

try:
    from picamera2 import Picamera2
    import simplejpeg
except ImportError:
    print("Warning: picamera2 or simplejpeg not found. Hardware camera won't start here.")

class VisionNode(MQTTNode):
    def __init__(self):
        super().__init__("VisionNode")
        self.processor = ArucoProcessor()
        self.latest_frame = None
        self.cond = Condition()
        self.aruco_enabled = True

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode('utf-8')
        if topic == "robobot/vision/enable":
            self.aruco_enabled = (payload == "1")

    def run(self):
        print(f"[{self.node_name}] Starting Picamera2 loop...")
        
        try:
            picam2 = Picamera2()
            picam2.configure(picam2.create_video_configuration(main={"size": (820, 616)}, controls={'FrameDurationLimits': (200000, 500000)}))
            picam2.start()
        except Exception as e:
            print(f"[{self.node_name}] Camera init failed: {e}")
            while self.running:
                time.sleep(1)
            return

        while self.running:
            frame = picam2.capture_array()
            if frame.shape[2] == 4:
                frame = frame[:, :, :3]
            
            if self.aruco_enabled:
                # 1. Process Frame
                res_frame, aruco_data = self.processor.process_image(frame)
                
                # 2. Publish numerical results continuously
                if aruco_data["markers"] or aruco_data["cube_center"] is not None:
                    self.client.publish("robobot/vision/aruco", json.dumps(aruco_data))

                # 3. Publish jpeg frame over MQTT or just let it be accessible for web stream
                # To avoid spamming MQTT with huge jpegs, we can host the pure stream server right here
                # Or wait, true ROS architecture sends Image messages over the bus...
                # For simplicity, we just publish the data here. The web ui logic follows below.
                
            time.sleep(0.01)
            
        picam2.stop()

def start_vision():
    node = VisionNode()
    node.start()

