with open("mqtt_python/aruco.py", "r") as f:
    content = f.read()

old_scipy_block = """                        # Marker relative Euler orientation (in Robot frame)
                        # e.g., Z, Y, X rotations
                        import scipy.spatial.transform as st
                        rot = st.Rotation.from_matrix(R_rob2marker)
                        m_euler = rot.as_euler('zyx', degrees=False) # yaw, pitch, roll
                        marker_obj["yaw"] = round(float(m_euler[0]), 2)
                        marker_obj["pitch"] = round(float(m_euler[1]), 2)
                        marker_obj["roll"] = round(float(m_euler[2]), 2)"""

new_math_block = """                        # Marker relative Euler orientation (in Robot frame) using numpy
                        sy = np.sqrt(R_rob2marker[0,0] * R_rob2marker[0,0] +  R_rob2marker[1,0] * R_rob2marker[1,0])
                        singular = sy < 1e-6
                        if not singular:
                            m_roll = np.arctan2(R_rob2marker[2,1], R_rob2marker[2,2])
                            m_pitch = np.arctan2(-R_rob2marker[2,0], sy)
                            m_yaw = np.arctan2(R_rob2marker[1,0], R_rob2marker[0,0])
                        else:
                            m_roll = np.arctan2(-R_rob2marker[1,2], R_rob2marker[1,1])
                            m_pitch = np.arctan2(-R_rob2marker[2,0], sy)
                            m_yaw = 0
                            
                        marker_obj["yaw"] = round(float(m_yaw), 2)
                        marker_obj["pitch"] = round(float(m_pitch), 2)
                        marker_obj["roll"] = round(float(m_roll), 2)"""

old_scipy_block2 = """                        # Robot Yaw (around Z of world)
                        rot_rob = st.Rotation.from_matrix(R_world2rob)
                        robot_yaw_est = rot_rob.as_euler('zyx', degrees=False)[0]"""

new_math_block2 = """                        # Robot Yaw (around Z of world)
                        robot_yaw_est = np.arctan2(R_world2rob[1,0], R_world2rob[0,0])"""

content = content.replace(old_scipy_block, new_math_block)
content = content.replace(old_scipy_block2, new_math_block2)

with open("mqtt_python/aruco.py", "w") as f:
    f.write(content)
print("Removed scipy dependency")
