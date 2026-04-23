import time
import sys
import os
import cv2
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from nodes.mqtt_base import MQTTNode
from lib.vision.shared_mem import SharedImageReader, SharedImageWriter

class BWThresholdNode(MQTTNode):
    def __init__(self):
        super().__init__("BWThresholdNode")
        self.reader = SharedImageReader("raw_frame", (616, 820, 3))
        self.writer = SharedImageWriter("overlay_bw", (616, 820, 4))

    def run(self):
        print(f"[{self.node_name}] Waiting for camera...")
        while self.running:
            raw_frame = self.reader.read()
            if raw_frame is not None:
                break
            time.sleep(0.5)

        print(f"[{self.node_name}] Starting BW Threshold loop...")
        while self.running:
            raw_frame = self.reader.read()
            if raw_frame is None:
                time.sleep(0.01)
                continue

            try:
                # Grayscale
                gray = cv2.cvtColor(raw_frame, cv2.COLOR_RGB2GRAY)
                # Binary threshold at 127
                _, bw = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

                # Output requires an RGBA image. Map grayscale back to 3 channels + alpha 255
                # Since an opaque BW image blocks everything below it, its alpha is always 255
                bw_rgb = cv2.cvtColor(bw, cv2.COLOR_GRAY2RGB)
                
                # Append Alpha Channel (255 everywhere)
                alpha_channel = np.ones((616, 820, 1), dtype=np.uint8) * 255
                overlay_rgba = np.concatenate((bw_rgb, alpha_channel), axis=2)

                self.writer.write(overlay_rgba)
            except Exception as e:
                print(f"[BWThresholdNode] Error: {e}")
            time.sleep(0.03)

        self.writer.cleanup()

def start_bw_threshold():
    node = BWThresholdNode()
    node.start()
