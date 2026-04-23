"""
The ball detection node detects the ball in the camera feed and publishes its position to the rest of the system on MQTT. It also provides the data for the visual overlay of the ball in the stream node.
"""

import json
import time
import numpy as np
import cv2 as cv
from multiprocessing import shared_memory
from biscaROS.core.base_node import BaseNode

class BallNode(BaseNode):
    def __init__(self):
        super().__init__(node_name="ball_detector_node")
        
        self.frame_shape = (616, 820, 3)
        self.frame_size = int(np.prod(self.frame_shape))
        
        # 1. Attach to Camera SHM (Input)
        try:
            self.cam_shm = shared_memory.SharedMemory(name="biscaROS_camera_frame")
            self.cam_array = np.ndarray(self.frame_shape, dtype=np.uint8, buffer=self.cam_shm.buf)
        except FileNotFoundError:
            print("[BallNode] Error: Camera shared memory not found. Start camera node first.")
            exit(1)
            
        # 2. Create Overlay SHM (Output)
        try:
            self.overlay_shm = shared_memory.SharedMemory(name="biscaROS_ball_overlay", create=True, size=self.frame_size)
        except FileExistsError:
            self.overlay_shm = shared_memory.SharedMemory(name="biscaROS_ball_overlay", create=False)
            
        self.overlay_array = np.ndarray(self.frame_shape, dtype=np.uint8, buffer=self.overlay_shm.buf)
        
        # 3. Initialize Homography Matrix
        self.H_matrix = self._setup_homography()
        
        # Subscribe to the camera doorbell
        self.client.subscribe("biscaROS/vision/camera/new_frame")

    def _setup_homography(self):
        # Using the hardcoded calibration points from golf.py
        src_pts = np.array([ 
            [213, 304],  # Middle left pixel
            [601, 307],  # Middle right pixel
            [45, 431],   # Bottom left pixel
            [774, 437]   # Bottom right pixel
        ], dtype=np.float32)

        dst_pts = np.array([ 
            [-0.52, 1.75], # Middle left CAD
            [0.52, 1.75],  # Middle right CAD
            [-0.31, 0.55], # Bottom left CAD
            [0.31, 0.55]   # Bottom right CAD
        ], dtype=np.float32)

        return cv.getPerspectiveTransform(src_pts, dst_pts)

    def _px_to_xy_homography(self, px_x, px_y):
        # OpenCV perspectiveTransform expects a 3D array
        pt = np.array([[[px_x, px_y]]], dtype=np.float32)
        transformed_pt = cv.perspectiveTransform(pt, self.H_matrix)
        
        real_x = float(transformed_pt[0][0][0])
        real_y = float(transformed_pt[0][0][1])
        
        target_distance = float(np.sqrt(real_x**2 + real_y**2))
        target_angle = float(np.arctan2(-real_x, real_y))
        
        return target_distance, target_angle, real_x, real_y

    def _on_message(self, client, userdata, msg):
        if msg.topic != "biscaROS/vision/camera/new_frame":
            return

        # 1. Read frame and prepare black canvas for additive compositing
        frame_rgb = np.copy(self.cam_array)
        frame_bgr = cv.cvtColor(frame_rgb, cv.COLOR_RGB2BGR)
        overlay_bgr = np.zeros_like(frame_bgr) # Black canvas for stream node
        
        # 2. Detect Red Ball (HSV Thresholding)
        hsv = cv.cvtColor(frame_bgr, cv.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv.inRange(hsv, lower_red2, upper_red2)
        combined_mask = cv.bitwise_or(mask1, mask2)
        
        eroded_mask = cv.erode(combined_mask, None, iterations=2)
        final_mask = cv.dilate(eroded_mask, None, iterations=2)
        
        contours, _ = cv.findContours(final_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        best_contour = None
        best_radius = 0
        best_center = (0, 0)
        max_area = 0
        img_height = frame_bgr.shape[0]
        
        for contour in contours:
            area = cv.contourArea(contour)
            if area > 10:
                ((x, y), radius) = cv.minEnclosingCircle(contour)
                circle_area = np.pi * (radius ** 2)
                
                if circle_area > 0:
                    area_ratio = area / circle_area
                    if area_ratio > 0.8 and area > max_area and y > img_height / 2:
                        max_area = area
                        best_contour = contour
                        best_radius = radius
                        best_center = (int(x), int(y))
        
        # 3. Process detection results
        ball_data = {"found": False}
        
        if best_contour is not None and best_radius > 5:
            px_x, px_y = best_center
            
            # Draw on the BLACK CANVAS
            cv.circle(overlay_bgr, (px_x, px_y), int(best_radius), (0, 255, 0), 2)
            cv.circle(overlay_bgr, (px_x, px_y), 3, (0, 0, 255), -1)
            
            # Compute real-world coordinates
            dist, angle, real_x, real_y = self._px_to_xy_homography(px_x, px_y)
            
            ball_data = {
                "found": True,
                "pixel_x": px_x,
                "pixel_y": px_y,
                "radius": float(best_radius),
                "real_x": real_x,
                "real_y": real_y,
                "target_distance": dist,
                "target_angle": angle
            }
        
        # 4. Publish math results
        self.publish("biscaROS/vision/ball/data", json.dumps(ball_data))
        
        # 5. Convert black canvas overlay back to RGB and save to shared memory
        overlay_rgb = cv.cvtColor(overlay_bgr, cv.COLOR_BGR2RGB)
        np.copyto(self.overlay_array, overlay_rgb)
        
        # Notify stream node that the ball overlay is ready
        self.publish("biscaROS/vision/overlays/ready", "biscaROS_ball_overlay")

    def stop(self):
        self.overlay_shm.close()
        self.overlay_shm.unlink()
        super().stop()

if __name__ == "__main__":
    node = BallNode()
    node.start()
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        node.stop()