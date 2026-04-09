with open("stream_server/stream_server.py", "r") as f:
    content = f.read()

old_block = """        elif self.path == '/api/aruco/data':
            response = {
                'enabled': aruco_enabled,
                'markers': aruco_data.get('markers', []) if isinstance(aruco_data, dict) else aruco_data,
                'cube_center': aruco_data.get('cube_center', None) if isinstance(aruco_data, dict) else None
            }"""
            
new_block = """        elif self.path == '/api/aruco/data':
            # Also send estimates
            est = aruco_data.get('estimates', []) if isinstance(aruco_data, dict) else []
            
            response = {
                'enabled': aruco_enabled,
                'markers': aruco_data.get('markers', []) if isinstance(aruco_data, dict) else aruco_data,
                'cube_center': aruco_data.get('cube_center', None) if isinstance(aruco_data, dict) else None,
                'estimates': est
            }"""

if old_block in content:
    content = content.replace(old_block, new_block)
    with open("stream_server/stream_server.py", "w") as f:
        f.write(content)
        print("stream_server.py patched successfully")
else:
    print("Could not patch stream_server.py")
