# bobROS Architecture: Copilot Onboarding Guide

Hello, fellow Copilot! You are assisting with a Raspberry Pi + Teensy-based robotics project. The codebase has recently undergone a major architectural refactor from a tightly-coupled monolithic script to a cleanly abstracted, multi-process, ROS-lite architecture named **bobROS**. 

This document explains the historical context, the new architectural paradigm, and the strict rules for adding entirely new missions or features.

## 1. The Situation & Hardware Context
- **Hardware**: The robot uses a Raspberry Pi for high-level logic (Vision, Planning, Web UI) and a Teensy microcontroller for hard real-time control (Motors, IMU reading, Encoders).
- **Bridge**: A C++ executable (`teensy_interface`) runs on the Pi to talk to the Teensy via serial, exposing the hardware layer entirely through **MQTT topics** (`robobot/drive/...`, `robobot/cmd/...`).

## 2. The Previous Architecture (Legacy `robobot/` folder)
*Do not write code matching this old style.*
- **The "God Object"**: A monolithic script (`uservice.py`) handled the MQTT connection and blindly forwarded payloads to global singleton instances of every module (`spose.py`, `sfuse.py`, `sedge.py`).
- **Global State Contamination**: Variables were shared globally across files. If the camera failed, the whole navigation stack could crash.
- **Blocking Mission Loops**: Missions (e.g., `cube_catch.py`) ran inside infinite `while` loops using blocking `time.sleep()` calls, fighting for CPU time.
- **Spaghetti Web Server**: `stream_server.py` ran an HTTP server, processed ArUco vision, and literally used `subprocess.Popen` to launch Python mission scripts when a web button was clicked.

## 3. The New Architecture (`bobROS/` folder)
The system is now a **Multi-Process ROS-Lite Architecture** utilizing Python's `multiprocessing` to bypass the GIL, with MQTT as the sole Inter-Process Communication (IPC) bus. 

### Directory Tree & Abstraction Layers
```text
bobROS/
├── main.py                         # The central supervisor script (Entry Point)
│
├── lib/                            # PURE ALGORITHMS (No MQTT, No loops)
│   ├── estimation/kalman.py        # Abstract EKF math (previously sfuse.py)
│   └── vision/aruco.py             # Pure OpenCV ArUco detector
│
├── nodes/                          # IPC WRAPPERS (Handles MQTT and process loops)
│   ├── mqtt_base.py                # Abstract base class for all nodes
│   ├── mission.py                  # The brain: listens to UI commands, executes Task arrays
│   ├── navigation.py               # Feeds IMU/Encoder MQTT data into lib/estimation/kalman.py
│   ├── vision.py                   # Feeds camera frames into lib/vision/aruco.py
│   └── web.py                      # Serves web_assets and bridges REST <-> MQTT
│
├── missions/                       # MISSION LOGIC (Non-blocking)
│   ├── tasks.py                    # Atomic, reusable Task classes
│   └── cube_catch.py               # Sequence definitions (arrays of Tasks)
│
├── web_assets/                     # FRONTEND
│   └── index.html, main.js         # Pure static UI
│
└── teensy_interface/               # HARDWARE BRIDGE (Compiled C++)
```

### Key Paradigms to Follow:
1. **MQTT is the ONLY Boundary**: Nodes must *never* import variables from other nodes. If `vision.py` needs to tell `mission.py` where a cube is, it publishes to `robobot/vision/aruco` as JSON.
2. **Processes, not Threads**: `main.py` spawns each node in `nodes/` as an isolated OS process. If OpenCV locks up a CPU core, the Kalman filter on another core keeps running flawlessly.
3. **Libraries are agnostic**: Files in `lib/` must never contain `paho.mqtt` or `multiprocessing`. They are pure Data-In -> Data-Out math classes.

## 4. How to Modify and Extend

### A. How to Add a New Mission
Missions are no longer blocking `while` loops. They are **Task Arrays**.
1. **Create the Tasks**: If a new behavior is needed (e.g., Follow Line), create a new class inheriting from `Task` in `missions/tasks.py`. It requires `start()` and non-blocking `update()` logic.
2. **Define the Sequence**: Create a file in `missions/` (e.g., `golf.py`) returning an array of `Task` objects.
   ```python
   def get_golf_mission():
       return [
           TaskSearchArUco(marker_id=10),
           TaskApproachPoint(margin=0.2),
           TaskDrive(forward=0.2, turn=0.0, duration=2.0)
       ]
   ```
3. **Hook it up**: Edit `nodes/mission.py` to listen for a specific MQTT command string from the UI and assign `self.current_mission = get_golf_mission()`.

### B. How to Add a New Hardware/Sensor Feature
1. **Add the Math**: Write the pure algorithmic interpretation in `lib/`.
2. **Add a Node**: Create `nodes/sensor_name.py` inheriting from `MQTTNode`. Setup the `run()` loop to read the sensor, apply the math, and publish to MQTT.
3. **Register Process**: Add the new node to the `processes` dictionary in exactly one place: `main.py`.

### C. UI Changes
- Edit HTML/JS in `web_assets/`.
- To trigger robot actions from the UI, make an HTTP request to the `web.py` node (e.g., `/api/mission/start`), which translates it directly into an MQTT publish on `robobot/cmd/mission`.

### D. Hot-Reloading and Restarting Nodes (Iterative Development)
**Do you need to restart the whole `main.py` if you only edit a mission?**
NO! Because bobROS is a decoupled, multi-process architecture communicating purely via MQTT, you can stop and restart *just* the `mission.py` node without affecting the rest of the system.
- The `web.py` node stays alive, meaning the website never disconnects and you don't lose your UI session.
- The `vision.py` node keeps processing and streaming camera frames.
- The `navigation.py` node keeps tracking the robot's pose seamlessly relying on the Teensy.

**How to leverage this / Implement a "Refresh" button on the website:**
Yes, a "Refresh Node" button on the website is completely possible and highly encouraged for rapid development!
To implement this:
1. The website UI sends an HTTP request (e.g., `POST /api/sys/restart?node=mission`) to `web.py`.
2. `web.py` translates this into an MQTT message published to a system topic like `robobot/sys/restart` with payload `{"node": "Mission_Control"}`.
3. `main.py` (the orchestrator) is updated to include a lightweight MQTT client of its own. When it receives this message, it calls `.terminate()` on that specific child process, re-imports the target module, and spawns a brand new `.start()` process.
This guarantees that the modified Python code from the disk is loaded completely fresh, all without dropping the camera stream or disconnecting the user from the web UI!

Alternatively, during quick terminal development, you can tell `main.py` to not start the mission node, and manually run `python3 -m nodes.mission` in a separate terminal. This way, you can hit `Ctrl+C` and restart just the mission logic in seconds!

**Good luck, Copilot! Stay modular, keep it non-blocking, and respect the MQTT boundaries.**