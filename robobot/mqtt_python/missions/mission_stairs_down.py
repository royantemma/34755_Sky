"""
mission_stairs_down.py
-----------------------
Mission to go down 5 stairs and follow line to intersection.

This mission:
1. Drives forward fast until detecting an incline (pitch != 0)
2. For each stair (5 times):
   - Drives slowly while following the line for 10cm
   - Accelerates when back to horizontal (pitch ~= 0)
3. Follows the line until an intersection is detected

The stairs_down task automatically handles the pitch detection and
line following with configurable parameters.
"""

TOTAL_TIME       = 120  # Total time allowed for the mission
GOAL_TIME_BUFFER = 10   # Time buffer before going to goal

TASKS = [
    # Servo down to activate line sensor
    {
        "type": "servo_mid",
    },
    
    # Reset IWO at starting position
    {
        "type": "reset_IWO",
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "roll": 0,
        "pitch": 0,
        "yaw": 0,
    },
    
    # Main task: go down stairs
    {
        "type": "stairs_down",
        "side": "left",                 # Follow left edge of line
        "fast_speed": 0.4,              # Speed when horizontal (m/s) - reduced for stability
        "slow_speed": 0.12,             # Speed when descending (m/s) - reduced for gentler control
        "step_distance": 0.08,          # Distance per stair (meters) - reduced for better control
        "pitch_threshold": 0.08,        # Pitch change to detect stair (radians, ~4.6°) - reduced for sensitivity
        "num_stairs": 5,                # Number of stairs to descend
        "timeout": 60,                  # Timeout for entire task (seconds)
    },
    
    # Servo up to deactivate line sensor
    {
        "type": "servo_up",
    },
]
