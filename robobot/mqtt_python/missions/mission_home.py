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

TOTAL_TIME       = 300  # Total time allowed for the mission
GOAL_TIME_BUFFER = 10   # Time buffer before going to goal

TASKS = [
    ##### 
    # Reset IWO at starting position
    {
    "type": "linefollow_until_x_intersections",
    "side": "left",
    "timeout": 30,
    "speed": 0.6,
    },
    {
        "type": "reset_IWO",
        "x": 2.48,
        "y": 3.48,
        "z": 0.0,
        "roll": 0,
        "pitch": 0,
        "yaw": 180,
    },

    {
        "type": "drive_to_xyyaw_IWO",
        "target_x": 0.48,
        "target_y": 3.48,
        "target_yaw": -90,
        "max_speed": 0.8,
        "distance_tolerance": 0.05
    },
    {
        "type": "drive_to_xyyaw_IWO",
        "target_x": 0.48,
        "target_y": 1.4,
        "target_yaw":-80,
        "max_speed": 0.8,
        "distance_tolerance": 0.05
    },
    {
    "type": "drive_until_line_detection",
    "speed": 0.4,
    "timeout": 30
    },
    {
        "type":     "drive_straight",
        "start_speed": 0.25,
        "speed":    0.25,
        "distance": 0.15,
        "timeout":  200,
        "is_stopping_at_end": True
    },
    {
        "type": "drive_to_xyyaw_IWO",
        "target_yaw": -70, # 0 is good for small speeds, but -5 is for when the robot overshoots
        "just_yaw": True
    },

    {
    "type": "drive_until_line_detection",
    "speed": 0.4,
    "timeout": 30
    },

    {
        "type": "drive_to_xyyaw_IWO",
        "target_yaw": 0, # 0 is good for small speeds, but -5 is for when the robot overshoots
        "just_yaw": True
    },
    {
        "type": "line_follow_vision", # controlling speed ? stops ? crossings ?
        "timeout": 20,
        "junction": 'right',
        "start_speed": 0,
        "nominal_speed": 0.6,
        "stop_speed": 0.25,
        "turn_sensitivity": 0.005,
        "time_to_full_speed": 0.2,
        "stop_time": 0.4,
        "min_pixels_to_detect_line": 200
    }, 
    {
     "type":  "line_follow",
     "side": "left",
     "timeout": 7,
     "speed": 0.3,
},


    
    
]
