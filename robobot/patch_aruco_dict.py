with open("mqtt_python/aruco.py", "r") as f:
    content = f.read()

old_dict = """MARKER_DB = {
    # Static markers
    10: {'type': 'static', 'size': 0.1, 'x': 1240.0, 'y': -1770.0, 'z': 117.0, 'yaw': np.radians(45)},
    11: {'type': 'static', 'size': 0.1, 'x': 1240.0, 'y': 1170.0, 'z': 117.0, 'yaw': np.radians(-45)},
    12: {'type': 'static', 'size': 0.1, 'x': 1170.0, 'y': 1240.0, 'z': 117.0, 'yaw': np.radians(135)},
    13: {'type': 'static', 'size': 0.1, 'x': -1170.0, 'y': 1240.0, 'z': 117.0, 'yaw': np.radians(45)},
    14: {'type': 'static', 'size': 0.1, 'x': -1240.0, 'y': 1170.0, 'z': 117.0, 'yaw': np.radians(225)},
    15: {'type': 'static', 'size': 0.1, 'x': -1240.0, 'y': -1170.0, 'z': 117.0, 'yaw': np.radians(135)},
    16: {'type': 'static', 'size': 0.1, 'x': -1170.0, 'y': -1240.0, 'z': 117.0, 'yaw': np.radians(-45)},
    17: {'type': 'static', 'size': 0.1, 'x': 1170.0, 'y': -1240.0, 'z': 117.0, 'yaw': np.radians(225)},"""

new_dict = """MARKER_DB = {
    # Static markers
    10: {'type': 'static', 'size': 0.1, 'x': 1240.0, 'y': -1770.0, 'z': 117.0, 'yaw': np.radians(135)},
    11: {'type': 'static', 'size': 0.1, 'x': 1240.0, 'y': 1170.0, 'z': 117.0, 'yaw': np.radians(45)},
    12: {'type': 'static', 'size': 0.1, 'x': 1170.0, 'y': 1240.0, 'z': 117.0, 'yaw': np.radians(-135)},
    13: {'type': 'static', 'size': 0.1, 'x': -1170.0, 'y': 1240.0, 'z': 117.0, 'yaw': np.radians(135)},
    14: {'type': 'static', 'size': 0.1, 'x': -1240.0, 'y': 1170.0, 'z': 117.0, 'yaw': np.radians(-45)},
    15: {'type': 'static', 'size': 0.1, 'x': -1240.0, 'y': -1170.0, 'z': 117.0, 'yaw': np.radians(-135)},
    16: {'type': 'static', 'size': 0.1, 'x': -1170.0, 'y': -1240.0, 'z': 117.0, 'yaw': np.radians(45)},
    17: {'type': 'static', 'size': 0.1, 'x': 1170.0, 'y': -1240.0, 'z': 117.0, 'yaw': np.radians(-45)},"""

old_math = """                        # Build rotation matrix of marker in world frame
                        # Based on: Z is out, Y is down, X is right.
                        # If the marker Y axis is the world yaw axis (Z_world), we rotate around X by -90 or 90
                        # Let's say marker is placed upright on a wall: Z_m points OUT, Y_m points UP (which is Z_world), X_m points HORIZONTALLY right
                        R_base = np.array([
                            [ 1,  0,  0 ],
                            [ 0,  0,  1 ],
                            [ 0, -1,  0 ]
                        ]) # Here X_m -> X_w, Y_m -> Z_w, Z_m -> -Y_w
                        
                        R_yaw = np.array([
                            [np.cos(world_yaw), -np.sin(world_yaw), 0],
                            [np.sin(world_yaw),  np.cos(world_yaw), 0],
                            [0, 0, 1]
                        ])
                        R_world2marker = R_yaw @ R_base"""

new_math = """                        # Build rotation matrix of marker in world frame
                        # Base marker at origin: Z out, X right, Y down.
                        # "movement applied is a certain angle in yaw, then roll of +90 for all of them"
                        
                        # Roll of +90 around X axis
                        R_roll = np.array([
                            [1, 0,  0],
                            [0, 0, -1],
                            [0, 1,  0]
                        ])
                        
                        # Yaw around Z axis
                        R_yaw = np.array([
                            [np.cos(world_yaw), -np.sin(world_yaw), 0],
                            [np.sin(world_yaw),  np.cos(world_yaw), 0],
                            [ 0,                 0,                 1]
                        ])
                        
                        # Intrinsic rotation: Apply Yaw, then Roll (+90)
                        R_world2marker = R_yaw @ R_roll"""

content = content.replace(old_dict, new_dict).replace(old_math, new_math)

with open("mqtt_python/aruco.py", "w") as f:
    f.write(content)

print("aruco.py yaw array and math transforms replaced")
