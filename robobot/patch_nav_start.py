with open("stream_server/stream_server.py", "r") as f:
    content = f.read()

import os

old_block = """def start_stream_server():
    threading.Thread(target=process_frames, daemon=True).start()
    threading.Thread(target=aruco_worker, daemon=True).start()"""
    
new_block = """def start_stream_server():
    import subprocess
    mqtt_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "mqtt_python")
    subprocess.Popen(["python3", "navigation.py"], cwd=mqtt_dir)
    threading.Thread(target=process_frames, daemon=True).start()
    threading.Thread(target=aruco_worker, daemon=True).start()"""

if old_block in content:
    content = content.replace(old_block, new_block)
    with open("stream_server/stream_server.py", "w") as f:
        f.write(content)
        print("stream_server.py patched to launch nav")
else:
    print("Could not patch start_stream_server")
