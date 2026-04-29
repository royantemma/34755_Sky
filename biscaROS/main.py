"""
Instantiate the bisacaROS nodes and start the event loop.
"""

# biscaROS/main.py
import subprocess
import sys
import time
import signal
import os

# To disable a node, simply comment out its line.
# Format: ("path/to/script.py", delay_after_start_in_seconds)
NODES = [
    # # --- Hardware Nodes ---
    # ("biscaROS/nodes/hardware/imu_node.py", 0.1),
    # ("biscaROS/nodes/hardware/ir_node.py", 0.1),
    # ("biscaROS/nodes/hardware/robot_node.py", 0.1),

    # # --- Core Nodes ---
    # ("biscaROS/nodes/navigation_node.py", 0.1),  
    # ("biscaROS/nodes/mission_node.py", 0.1),

    # # --- Vision Nodes ---
    # # CRITICAL: Camera node requires a delay to allocate Shared Memory
    # # before the other vision nodes attempt to attach to it.
    ("biscaROS/nodes/vision/camera_node.py", 2.5),
    
    ("biscaROS/nodes/vision/aruco_node.py", 0.1),
    ("biscaROS/nodes/vision/ball_node.py", 0.1),
    
    # # Stream compositor must start after all overlays are available
    ("biscaROS/nodes/vision/stream_node.py", 0.1),

    # # --- Web/UI Nodes ---
    ("biscaROS/nodes/web_node.py", 0.1),
]

processes = []

def signal_handler(sig, frame):
    print("\n[main.py] Shutting down all biscaROS nodes...")
    for p in processes:
        p.terminate() # Standard termination
    
    time.sleep(1) # Give processes 1 second to clean up memory
    
    for p in processes:
        if p.poll() is None: # If process is still alive
            print(f"[main.py] Force killing stuck process PID {p.pid}")
            p.kill() # SIGKILL
            
    print("[main.py] System shutdown complete.")
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    print("[main.py] Booting biscaROS architecture...")
    
    # Capture the current environment and append the current working directory
    current_env = os.environ.copy()
    current_env["PYTHONPATH"] = os.getcwd() + os.pathsep + current_env.get("PYTHONPATH", "")
    
    for script, delay in NODES:
        print(f"[main.py] Launching {script}...")
        try:
            # Pass the modified environment to the subprocess
            p = subprocess.Popen([sys.executable, script], env=current_env)
            processes.append(p)
            time.sleep(delay)
        except Exception as e:
            print(f"[main.py] FAILED to launch {script}: {e}")

    print("[main.py] All nodes launched successfully. Press Ctrl+C to terminate.")
    
    while True:
        time.sleep(1.0)