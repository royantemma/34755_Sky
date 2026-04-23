import time
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from nodes.mqtt_base import MQTTNode
from lib.vision.shared_mem import SharedImageWriter

class CameraNode(MQTTNode):
    def __init__(self):
        super().__init__("CameraNode")
        self.writer = SharedImageWriter("raw_frame", (616, 820, 3))

    def run(self):
        print(f"[{self.node_name}] Starting Picamera2 at 30 FPS...")
        
        try:
            from picamera2 import Picamera2
        except ImportError:
            print("[CameraNode] Warning: picamera2 not found. Emulating camera.")
            import numpy as np
            while self.running:
                frame = np.zeros((616, 820, 3), dtype=np.uint8)
                self.writer.write(frame)
                time.sleep(1/30)
            return

        try:
            picam2 = Picamera2()
            config = picam2.create_video_configuration(main={"size": (820, 616)}, controls={
                "ExposureTime": 15000, 
                "AnalogueGain": 4.0, 
                "FrameDurationLimits": (33333, 33333)
            })
            picam2.configure(config)
            picam2.start()
        except Exception as e:
            print(f"[{self.node_name}] Camera init failed: {e}")
            import traceback
            traceback.print_exc()
            while self.running:
                time.sleep(1)
            return

        while self.running:
            try:
                frame = picam2.capture_array()
                if frame.shape[2] == 4:
                    frame = frame[:, :, :3]
                
                self.writer.write(frame)
            except Exception as e:
                print(f"[CameraNode] Capture error: {e}")
                
            time.sleep(0.001) # Yield slightly
            
        picam2.stop()
        self.writer.cleanup()

def start_camera():
    node = CameraNode()
    node.start()
