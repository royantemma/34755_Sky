import sys

with open("mqtt_python/aruco.py", "r") as f:
    content = f.read()

# Replace the block representing the static marker processing
old_block = """                    elif m_type == 'static':
                        # Compute robot position
                        world_x, world_y, world_z, world_yaw = marker_info['x'], marker_info['y'], marker_info['z'], marker_info['yaw']
                        
                        R_rob2cam = self.R_cam2rob.T
                        T_rob2cam = -R_rob2cam @ self.T_cam2rob
                        
                        R_world2cam = R_marker
                        T_world2cam = tvec_mm
                        
                        # We want R_world2rob, T_world2rob to figure out where robot is in world.
                        R_world2rob = self.R_cam2rob @ R_world2cam
                        T_world2rob = (self.R_cam2rob @ T_world2cam) + self.T_cam2rob
                        
                        R_rob2world = R_world2rob.T
                        T_rob2world = -R_rob2world @ T_world2rob
                        
                        robot_x_est = world_x + T_rob2world[0,0]
                        robot_y_est = world_y + T_rob2world[1,0]
                        robot_yaw_est = world_yaw + np.arctan2(R_rob2world[1,0], R_rob2world[0,0])
                        
                        estimates.append({
                            "marker_id": marker_id,
                            "robot_x": round(float(robot_x_est), 1),
                            "robot_y": round(float(robot_y_est), 1),
                            "robot_yaw": float(robot_yaw_est)
                        })"""

new_block = """                    elif m_type == 'static':
                        # 1. Marker in Robot frame
                        R_rob2marker = self.R_cam2rob @ R_marker
                        T_rob2marker = marker_rob # which is self.R_cam2rob @ tvec_mm + self.T_cam2rob
                        
                        # Marker relative Euler orientation (in Robot frame)
                        # e.g., Z, Y, X rotations
                        import scipy.spatial.transform as st
                        rot = st.Rotation.from_matrix(R_rob2marker)
                        m_euler = rot.as_euler('zyx', degrees=False) # yaw, pitch, roll
                        marker_obj["yaw"] = round(float(m_euler[0]), 2)
                        marker_obj["pitch"] = round(float(m_euler[1]), 2)
                        marker_obj["roll"] = round(float(m_euler[2]), 2)
                        
                        # Compute robot position in world
                        world_x, world_y, world_z, world_yaw = marker_info['x'], marker_info['y'], marker_info['z'], marker_info['yaw']
                        T_world2marker = np.array([[world_x], [world_y], [world_z]])
                        
                        # Build rotation matrix of marker in world frame
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
                        R_world2marker = R_yaw @ R_base
                        
                        # Core Derivation:
                        # X_w = R_w2m * X_m + T_w2m
                        # X_r = R_r2m * X_m + T_r2m   =>  X_m = R_r2m^T * (X_r - T_r2m)
                        # So X_w = R_w2m * R_r2m^T * X_r  -  R_w2m * R_r2m^T * T_r2m + T_w2m
                        # X_w = R_w2r * X_r + T_w2r
                        
                        R_world2rob = R_world2marker @ R_rob2marker.T
                        T_world2rob = T_world2marker - (R_world2rob @ T_rob2marker)
                        
                        robot_x_est = T_world2rob[0, 0]
                        robot_y_est = T_world2rob[1, 0]
                        
                        # Robot Yaw (around Z of world)
                        rot_rob = st.Rotation.from_matrix(R_world2rob)
                        robot_yaw_est = rot_rob.as_euler('zyx', degrees=False)[0]
                        
                        estimates.append({
                            "marker_id": marker_id,
                            "robot_x": round(float(robot_x_est), 1),
                            "robot_y": round(float(robot_y_est), 1),
                            "robot_yaw": float(robot_yaw_est)
                        })"""

if old_block in content:
    content = content.replace(old_block, new_block)
    with open("mqtt_python/aruco.py", "w") as f:
        f.write(content)
    print("aruco.py successfully patched")
else:
    print("Could not find the block to patch")
