import time
import sys
import os
import json
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from nodes.mqtt_base import MQTTNode
from lib.vision.shared_mem import SharedImageReader, SharedImageWriter
from lib.vision.aruco import ArucoProcessor

class ArucoNode(MQTTNode):
    def __init__(self):
        super().__init__("ArucoNode")
        self.reader = SharedImageReader("raw_frame", (616, 820, 3))
        self.writer = SharedImageWriter("overlay_aruco", (616, 820, 4))
        self.processor = ArucoProcessor()

    def run(self):
        print(f"[{self.node_name}] Waiting for camera...")
        while self.running:
            raw_frame = self.reader.read()
            if raw_frame is not None:
                break
            time.sleep(0.5)

        print(f"[{self.node_name}] Starting ArUco detection loop...")
        while self.running:
            raw_frame = self.reader.read()
            if raw_frame is None:
                time.sleep(0.01)
                continue

            # Start with fully transparent RGBA array
            overlay = np.zeros((616, 820, 4), dtype=np.uint8)

            try:
                # Create a black 3-channel canvas for OpenCV compatibility
                black_canvas = np.zeros((616, 820, 3), dtype=np.uint8)
                
                # The processor puts markers on the black RGB canvas
                black_canvas, aruco_data = self.processor.process_image(raw_frame, black_canvas)
                
                if aruco_data["markers"] or aruco_data.get("cube_center") is not None:
                    self.client.publish("robobot/vision/aruco", json.dumps(aruco_data))

                # Transfer drawn pixels to the RGBA overlay
                # Find pixels that are not black
                alpha_mask = (black_canvas.sum(axis=2) > 0)
                overlay[alpha_mask, :3] = black_canvas[alpha_mask]
                overlay[alpha_mask, 3] = 255

                self.writer.write(overlay)
            except Exception as e:
                print(f"[{self.node_name}] Error: {e}")
            time.sleep(0.03)

        self.writer.cleanup()

def start_aruco():
    node = ArucoNode()
    node.start()
