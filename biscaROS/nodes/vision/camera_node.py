"""
The camera node is responsible for getting the images from the camera and publishing them to the rest of the system (on shared memory). It also calibrates the camera.
"""

import time
import numpy as np
from multiprocessing import shared_memory
from biscaROS.core.base_node import BaseNode

try:
    from picamera2 import Picamera2
except ImportError:
    print("[CameraNode] Picamera2 not found. Are you running this on the Pi?")
    exit(1)

class CameraNode(BaseNode):
    def __init__(self):
        super().__init__(node_name="camera_node")
        
        # Camera dimensions
        self.width = 820
        self.height = 616
        self.channels = 3 # RGB
        self.frame_shape = (self.height, self.width, self.channels)
        self.frame_size = self.width * self.height * self.channels
        self.shm_name = "biscaROS_camera_frame"
        
        # 1. Initialize Shared Memory
        try:
            self.shm = shared_memory.SharedMemory(name=self.shm_name, create=True, size=self.frame_size)
        except FileExistsError:
            # If the node crashed previously, the memory block might still exist. Attach to it.
            self.shm = shared_memory.SharedMemory(name=self.shm_name, create=False)
            
        # Create a NumPy array that directly writes into the shared memory block
        self.shared_array = np.ndarray(self.frame_shape, dtype=np.uint8, buffer=self.shm.buf)

        # 2. Initialize Camera
        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(main={"size": (self.width, self.height)}, controls={
            "ExposureTime": 15000, 
            "AnalogueGain": 4.0, 
            "FrameDurationLimits": (33333, 33333) # Forces ~30 fps
        })
        self.picam2.configure(config)
        self.picam2.start()

    def run(self):
        # We don't just use self.start() here because we need a blocking loop to capture frames
        super().start() # Starts the background MQTT thread
        frame_count = 0
        
        print("[CameraNode] Starting capture loop...")
        try:
            while True:
                # Capture frame (blocks until next frame is ready)
                frame = self.picam2.capture_array()
                
                # Copy the new frame data directly into the shared memory block
                np.copyto(self.shared_array, frame)
                
                # Ring the "MQTT Doorbell" so vision nodes know a new frame is ready
                timestamp = time.time()
                self.publish("biscaROS/vision/camera/new_frame", f"{frame_count},{timestamp}")
                
                frame_count += 1
                
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        print("[CameraNode] Shutting down, cleaning up shared memory...")
        self.picam2.stop()
        self.shm.close()
        self.shm.unlink() # CRITICAL: Releases the RAM back to the OS
        super().stop()

if __name__ == "__main__":
    node = CameraNode()
    node.run()