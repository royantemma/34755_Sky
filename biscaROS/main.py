"""
Instantiate the bisacaROS nodes and start the event loop.
"""

# biscaROS/main.py
import subprocess
import sys
import time
import signal

# To disable a node, simply comment out its line.
# Format: ("path/to/script.py", delay_after_start_in_seconds)
NODES = [
    # --- Hardware Nodes ---
    ("biscaROS/nodes/hardware/imu_node.py", 0.1),
    ("biscaROS/nodes/hardware/ir_node.py", 0.1),
    ("biscaROS/nodes/hardware/robot_node.py", 0.1),

    # --- Core Nodes ---
    ("biscaROS/nodes/navigation.py", 0.1),  
    ("biscaROS/nodes/mission_node.py", 0.1),

    # --- Vision Nodes ---
    # CRITICAL: Camera node requires a delay to allocate Shared Memory
    # before the other vision nodes attempt to attach to it.
    ("biscaROS/nodes/vision/camera_node.py", 2.5),
    
    ("biscaROS/nodes/vision/aruco_node.py", 0.1),
    # ("biscaROS/nodes/vision/ball_node.py", 0.1),
    
    # Stream compositor must start after all overlays are available
    ("biscaROS/nodes/vision/stream_node.py", 0.1),

    # --- Web/UI Nodes ---
    ("biscaROS/nodes/web_node.py", 0.1),
]

processes = []

def signal_handler(sig, frame):
    print("\n[main.py] Shutting down all biscaROS nodes...")
    for p in processes:
        p.send_signal(signal.SIGINT) # Send Ctrl+C to children for graceful shutdown
    
    # Wait for all processes to clean up (e.g., releasing shared memory)
    for p in processes:
        p.wait()
        
    print("[main.py] System shutdown complete.")
    sys.exit(0)

if __name__ == "__main__":
    # Register the shutdown handler
    signal.signal(signal.SIGINT, signal_handler)

    print("[main.py] Booting biscaROS architecture...")
    
    for script, delay in NODES:
        print(f"[main.py] Launching {script}...")
        try:
            # sys.executable ensures the subprocess uses the exact same Python 
            # virtual environment as this main script.
            p = subprocess.Popen([sys.executable, script])
            processes.append(p)
            time.sleep(delay)
        except Exception as e:
            print(f"[main.py] FAILED to launch {script}: {e}")

    print("[main.py] All nodes launched successfully. Press Ctrl+C to terminate.")
    
    # Keep the main thread alive to catch signals
    while True:
        time.sleep(1.0)