import time
import sys
import os
import cv2
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from nodes.mqtt_base import MQTTNode
from lib.vision.shared_mem import SharedImageReader, SharedImageWriter

class RedMaskNode(MQTTNode):
    def __init__(self):
        super().__init__("RedMaskNode")
        self.reader = SharedImageReader("raw_frame", (616, 820, 3))
        self.writer = SharedImageWriter("overlay_redmask", (616, 820, 4))
        self.enabled = True

    def on_message(self, client, userdata, msg):
        pass # Optional: we can toggle the processing logic entirely if we want

    def run(self):
        print(f"[{self.node_name}] Waiting for camera...")
        while self.running:
            raw_frame = self.reader.read()
            if raw_frame is not None:
                break
            time.sleep(0.5)

        print(f"[{self.node_name}] Starting RedMask detection loop...")
        while self.running:
            raw_frame = self.reader.read()
            if raw_frame is None:
                time.sleep(0.01)
                continue

            overlay = np.zeros((616, 820, 4), dtype=np.uint8)

            try:
                # Convert BGR/RGB to HSV
                # Assuming raw_frame is RGB right now
                hsv = cv2.cvtColor(raw_frame, cv2.COLOR_RGB2HSV)

                # Red wraps around Hue 0 and 180 in OpenCV (which uses 0-179 for hue)
                lower_red1 = np.array([0, 100, 100])
                upper_red1 = np.array([10, 255, 255])
                lower_red2 = np.array([160, 100, 100])
                upper_red2 = np.array([179, 255, 255])

                mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
                mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
                mask = cv2.bitwise_or(mask1, mask2)

                # Fill the red mask pixels with opaque Blue (RGBA)
                overlay[mask > 0] = [0, 0, 255, 255] # Solid blue overlay with full alpha

                self.writer.write(overlay)
            except Exception as e:
                print(f"[RedMaskNode] Error: {e}")
            time.sleep(0.03)

        self.writer.cleanup()

def start_red_mask():
    node = RedMaskNode()
    node.start()
