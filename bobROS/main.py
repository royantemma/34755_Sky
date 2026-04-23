import multiprocessing
import time
import sys
import subprocess
import os

os.environ["LIBCAMERA_LOG_LEVELS"] = "ERROR"

from nodes.vision.camera import start_camera
from nodes.vision.stream import start_stream
from nodes.vision.aruco import start_aruco
from nodes.vision.red_mask import start_red_mask
from nodes.vision.bw_threshold import start_bw_threshold

from nodes.mission import start_mission
from nodes.navigation import start_navigation
from nodes.web import start_web

def start_teensy_interface():
    path_to_build = os.path.join(os.path.dirname(__file__), "teensy_interface", "build2")

    cmd = ["./teensy_interface", "-d", "-l"]
    print(f"[Main] Starting teensy_interface in {path_to_build}...")
    subprocess.run(cmd, cwd=path_to_build)

if __name__ == "__main__":
    print("[Main] Initializing bobROS Architecture (Option B: Layered Vision)...")

    # Define processes
    processes = {
        #"TeensyInterface": multiprocessing.Process(target=start_teensy_interface),
        "Camera": multiprocessing.Process(target=start_camera),
        "Stream": multiprocessing.Process(target=start_stream),
        "Aruco": multiprocessing.Process(target=start_aruco),
        "RedMask": multiprocessing.Process(target=start_red_mask),
        "BWThreshold": multiprocessing.Process(target=start_bw_threshold),
        
        "Navigation": multiprocessing.Process(target=start_navigation),
        "Mission_Control": multiprocessing.Process(target=start_mission),
        "Web_Interface": multiprocessing.Process(target=start_web)
    }

    # Start processes
    for name, p in processes.items():
        print(f"[Main] Spawning {name} process...")
        p.start()

    try:
        # Parent stays alive, can monitor process health via p.is_alive()
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n[Main] Shutdown signal received. Terminating nodes...")
        for name, p in processes.items():
            p.terminate() # Force kill the child process
            p.join()
        print("[Main] Safely shut down bobROS.")
        sys.exit(0)