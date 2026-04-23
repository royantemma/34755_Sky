import cv2 as cv
import numpy as np
import os

target = '/home/local/svn/robobot/mqtt_python/test_plank_alignment.py'
with open(target, 'r') as f:
    c = f.read()

old_code = """    src_pts = np.array([[w//2-200, h-50], [w//2+200, h-50], [w//2-200, h//2+80], [w//2+200, h//2+80]], dtype=np.float32)

    dst_pts = []
    zoom = 0.5 # map 1 pixel = 2 mm
    cx = w//2
    cy = h - 20 # Anchor point bottom
    for pt in src_pts:
        floor_pt = pix_to_floor(pt[0], pt[1])
        if floor_pt is not None:
           dst_pts.append([cx - floor_pt[1]*zoom, cy - floor_pt[0]*zoom])"""

new_code = """    # Map a much wider 'trapezoid' from the bottom of the image towards the horizon
    # Grabbing the bottom corners, and points closer to the horizontal vanishing point
    src_pts = np.array([
        [0, h-10],           # Bottom Left
        [w-1, h-10],         # Bottom Right
        [w//2 - 150, h//2],  # Center Left (further away)
        [w//2 + 150, h//2]   # Center Right (further away)
    ], dtype=np.float32)

    dst_pts = []
    zoom = 0.5 # map 1 pixel = 2 mm
    cx = w // 2
    cy = h - 10 # Anchor point bottom
    for pt in src_pts:
        floor_pt = pix_to_floor(pt[0], pt[1])
        if floor_pt is not None:
           # Map Y (left/right in world) to X in image, and X (fwd/back) to Y in image
           dst_pts.append([cx - floor_pt[1]*zoom, cy - floor_pt[0]*zoom])"""

c = c.replace(old_code, new_code)
with open(target, 'w') as f:
    f.write(c)

print("Patched bird's eye view.")
