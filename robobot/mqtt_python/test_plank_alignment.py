import cv2 as cv
import numpy as np
import os
import math
from aruco import get_processor

# Ensure the output directory exists
out_dir = "plank_detection_images"
os.makedirs(out_dir, exist_ok=True)

# Grab frame from web stream manually
import urllib.request
try:
    req = urllib.request.urlopen("http://127.0.0.1:7123/stream/main")
    bytes_arr = bytes()
    for chunk in req:
        bytes_arr += chunk
        a = bytes_arr.find(b'\xff\xd8')
        b = bytes_arr.find(b'\xff\xd9')
        if a != -1 and b != -1:
            jpg = bytes_arr[a:b+2]
            frame = cv.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv.IMREAD_COLOR)
            break
except Exception as e:
    print(f"Stream read failed: {e}. Trying test_frame.jpg")
    frame = cv.imread("test_frame.jpg")

if frame is None:
    print("No frame to process!")
    exit(1)

hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
# Green mask bounds: Hue 35-85 roughly for bright green
lower_green = np.array([35, 50, 50])
upper_green = np.array([85, 255, 255])
mask = cv.inRange(hsv, lower_green, upper_green)
cv.imwrite(os.path.join(out_dir, "debug_green_mask.jpg"), mask)

# --- Idea 1: Raycasting bottom edge ---
idea1_img = frame.copy()
h, w = mask.shape
bottom_points = []
for x in range(0, w, 5): 
    for y in range(h-1, 10, -1):
        if mask[y, x] > 0:
            bottom_points.append((x, y))
            cv.circle(idea1_img, (x, y), 2, (0, 0, 255), -1)
            break

# RANSAC naturally ignores the right kink
if len(bottom_points) > 10:
    pts = np.array(bottom_points, dtype=np.int32).reshape(-1, 1, 2)
    [vx, vy, x0, y0] = cv.fitLine(pts, cv.DIST_L1, 0, 0.01, 0.01)
    vx, vy, x0, y0 = float(vx[0]), float(vy[0]), float(x0[0]), float(y0[0])
    yaw_error = math.degrees(math.atan2(vy, vx))
    
    left_y = int((-x0*vy/vx) + y0)
    right_y = int(((w-x0)*vy/vx) + y0)
    cv.line(idea1_img, (0, left_y), (w, right_y), (255, 0, 0), 3)
    cv.putText(idea1_img, f"Idea 1: Yaw {yaw_error:.1f} deg", (20, 40), cv.FONT_HERSHEY_SIMPLEX, 0.8, (255,0,0), 2)

cv.imwrite(os.path.join(out_dir, "idea1_raycast.jpg"), idea1_img)


# --- Idea 2: Hough Transform ---
idea2_img = frame.copy()
edges = cv.Canny(mask, 50, 150)
lines = cv.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=80, maxLineGap=40)
valid_lines = []
if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = line[0]
        angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
        # Filter horizontal lines
        if abs(angle) < 25 or abs(angle) > 155:
            valid_lines.append(line[0])
            cv.line(idea2_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

    if valid_lines:
        avg_line = np.mean(valid_lines, axis=0).astype(int)
        x1, y1, x2, y2 = avg_line
        cv.line(idea2_img, (x1, y1), (x2, y2), (0, 0, 255), 4)
        yaw_error = math.degrees(math.atan2(y2 - y1, x2 - x1))
        cv.putText(idea2_img, f"Idea 2: Yaw {yaw_error:.1f} deg avg", (20, 40), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)

cv.imwrite(os.path.join(out_dir, "idea2_hough.jpg"), idea2_img)


# --- Idea 3: Bird's Eye View ---
idea3_img = frame.copy()
try:
    ap = get_processor()
    R = ap.R_cam2rob
    T = ap.T_cam2rob
    mtx = ap.mtx

    def pix_to_floor(u, v):
        inv_mtx = np.linalg.inv(mtx)
        dc = inv_mtx @ np.array([u, v, 1.0])
        den = R[2,0]*dc[0] + R[2,1]*dc[1] + R[2,2]*dc[2]
        if abs(den) < 1e-6: return None
        zc = -T[2,0] / den
        xc = zc * dc
        x_rob = (R @ xc).flatten() + T.flatten()
        return x_rob[0], x_rob[1] # X (fwd mm), Y (left mm)

    # Map a much wider 'trapezoid' from the bottom of the image towards the horizon
    # Grabbing the bottom corners, and points closer to the horizontal vanishing point
    src_pts = np.array([
        [0, h-10],           # Bottom Left
        [w-1, h-10],         # Bottom Right
        [w//2 - 150, h//2],  # Center Left (further away)
        [w//2 + 150, h//2]   # Center Right (further away)
    ], dtype=np.float32)

    dst_pts = []
    zoom = 0.2 # map 1 pixel = 5 mm (less zoomed in)
    bird_w, bird_h = 1000, 1000 # Create a large, fixed 1000x1000 canvas to ensure nothing gets cropped
    cx = bird_w // 2
    cy = bird_h - 50 # Anchor point at the bottom center of the new canvas
    for pt in src_pts:
        floor_pt = pix_to_floor(pt[0], pt[1])
        if floor_pt is not None:
           # Map Y (left/right in world) to X in image, and X (fwd/back) to Y in image
           dst_pts.append([cx - floor_pt[1]*zoom, cy - floor_pt[0]*zoom])

    if len(dst_pts) == 4:
        dst_pts = np.array(dst_pts, dtype=np.float32)
        H = cv.getPerspectiveTransform(src_pts, dst_pts)
        bird = cv.warpPerspective(frame, H, (bird_w, bird_h))
        cv.imwrite(os.path.join(out_dir, "idea3_birdseye_raw.jpg"), bird)
        
        bird_hsv = cv.cvtColor(bird, cv.COLOR_BGR2HSV)
        bird_mask = cv.inRange(bird_hsv, lower_green, upper_green)
        bird_display = bird.copy()

        b_bottom = []
        for x in range(0, bird_w, 10):
            for y in range(bird_h-1, 10, -1):
                if bird_mask[y, x] > 0:
                    b_bottom.append((x, y))
                    cv.circle(bird_display, (x, y), 2, (0, 0, 255), -1)
                    break
            
        if len(b_bottom) > 5:
            pts_b = np.array(b_bottom, dtype=np.int32).reshape(-1, 1, 2)
            [vxb, vyb, x0b, y0b] = cv.fitLine(pts_b, cv.DIST_L1, 0, 0.01, 0.01)
            vxb, vyb, x0b, y0b = float(vxb[0]), float(vyb[0]), float(x0b[0]), float(y0b[0])
            yaw_err = math.degrees(math.atan2(vyb, vxb))
            p1 = (int(x0b - vxb*400), int(y0b - vyb*400))
            p2 = (int(x0b + vxb*400), int(y0b + vyb*400))
            cv.line(bird_display, p1, p2, (255,0,0), 3)
            dist_mm = (cy - y0b) / zoom
            cv.putText(bird_display, f"Idea 3: Yaw {yaw_err:.1f} deg", (20, 40), cv.FONT_HERSHEY_SIMPLEX, 0.8, (255,0,0), 2)
            cv.putText(bird_display, f"Dist: {dist_mm[0]:.1f} mm", (20, 80), cv.FONT_HERSHEY_SIMPLEX, 0.8, (255,0,0), 2)

        cv.imwrite(os.path.join(out_dir, "idea3_birdseye_detection.jpg"), bird_display)
except Exception as e:
    print("Bird's eye failed:", e)

print("Done! Look in plank_detection_images/")
