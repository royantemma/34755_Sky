with open("mqtt_python/aruco.py", "r") as f:
    content = f.read()

# Swap frame drawing color logic
old_block1 = """    def process_image(self, frame, valid_ids=None):
        if self.mtx is None or self.dist is None:
            return frame, {"markers": [], "cube_center": None, "estimates": []}
            
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)"""

new_block1 = """    def process_image(self, frame, valid_ids=None):
        if self.mtx is None or self.dist is None:
            return frame, {"markers": [], "cube_center": None, "estimates": []}
            
        # Convert RGB to BGR for OpenCV drawing correctly
        frame_bgr = cv.cvtColor(frame, cv.COLOR_RGB2BGR)
        gray = cv.cvtColor(frame_bgr, cv.COLOR_BGR2GRAY)"""

old_block2 = """        if ids is not None:
            cv.aruco.drawDetectedMarkers(frame, corners, ids)
            for i in range(len(ids)):"""

new_block2 = """        if ids is not None:
            cv.aruco.drawDetectedMarkers(frame_bgr, corners, ids)
            for i in range(len(ids)):"""

old_block3 = """                if ret:
                    cv.drawFrameAxes(frame, self.mtx, self.dist, rvec, tvec, size)"""

new_block3 = """                if ret:
                    cv.drawFrameAxes(frame_bgr, self.mtx, self.dist, rvec, tvec, size)"""

old_block4 = """        aruco_data = {
            "markers": current_data,
            "cube_center": avg_cube_data,
            "estimates": estimates
        }
        
        return frame, aruco_data"""

new_block4 = """        aruco_data = {
            "markers": current_data,
            "cube_center": avg_cube_data,
            "estimates": estimates
        }
        
        frame_res = cv.cvtColor(frame_bgr, cv.COLOR_BGR2RGB)
        return frame_res, aruco_data"""

if old_block1 in content:
    content = content.replace(old_block1, new_block1).replace(old_block2, new_block2).replace(old_block3, new_block3).replace(old_block4, new_block4)
    with open("mqtt_python/aruco.py", "w") as f:
        f.write(content)
    print("aruco.py converted frame drawing logic")
else:
    print("Could not find start block")

# Now fix index.html
with open("stream_server/index.html", "r") as f:
    html = f.read()

import re
import sys
# Removing the visual coordinate note block using regex because spacing can vary
pattern = r'<p[^>]*>.*?<em>Note on visual coordinate axes:.*?<\/p>'
html = re.sub(pattern, '', html, flags=re.DOTALL)

with open("stream_server/index.html", "w") as f:
    f.write(html)
print("index.html note removed")
