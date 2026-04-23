"""
missions/mission_all_v0_0.py
---------------------
version 0.0

Route: Start -> line follow -> take left at junction -> go on roundabout -> use vision to go around roundabout -> exit roundabout North using heading and update position 
-> line follow until x,y coord (5410,-2000) using odometry -> go to extra time point 1 (in mm and robo coord (3309,-2364))-> return to line and continue line following up the ramp and back down
-> go to coord (290,-3583) -> back up to (1550,-3583) -> rotate left 90 deg -> drive forward to (1550,-409) -> line follow -> Goal
"""

TOTAL_TIME       = 120
GOAL_TIME_BUFFER = 20

TASKS = [

    {
        "type": "line_follow_vision", # controlling speed ? stops ? crossings ?
        "timeout": 20,
        "junction": 'left',
        "start_speed": 0,
        "nominal_speed": 0.6,
        "stop_speed": 0.3,
        "turn_sensitivity": 0.005,
        "time_to_full_speed": 0.2,
        "stop_time": 0.4,
        "min_pixels_to_detect_line": 200
    }, 

    {
        "type":     "drive_straight",
        "speed":    0.3,
        "distance": 0.45,
        "timeout":  200,
        "is_stopping_at_end": True
    },

    {
        "type":     "drive_curved",
        "speed":    -0.1,
        "turn_rate": 1,
        "heading_deg": -55,
        "timeout":  200,
        "tolerance": 4
    },


    { # Theo: I tried higher speeds, but I didn't managed to get it work well, so don't loose your time on this ;)
        "type": "roundabout_vision", # controlling speed ? stops ?
        "timeout": 4,
        "speed": 0.4, # 0.4 recommended
        "stop_speed": 0.4,
        "time_to_full_speed": 0.3,
        "Kp": 6, # 6 recommended
        "Ki": 0.02 # 0.2 recommended
    },

    {
        "type":     "drive_straight",
        "speed":    0.4,
        "distance": 0.2,
        "timeout":  200,
        "is_stopping_at_end": False
    },

    {
        "type": "line_follow_vision",
        "timeout": 8,
        "junction": 'right',
        "start_speed": 0.4, # choose same as speed in "roundabout_vision"
        "nominal_speed": 1, # nice value is 0.6
        "stop_speed": 0,
        "turn_sensitivity": 0.007, # nice value is 0.005
        "time_to_full_speed": 2.5,
        "stop_time": 0.2
    }, 

    {
        "type":     "drive_straight",
        "speed":    0.01,
        "distance": 0.2,
        "timeout":  200,
        "is_stopping_at_end": False
    },

    # {
    #     "type":      "turn_to_heading",
    #     "heading_deg": 30, # 0 is the heading at the beginning ot the mission, so this is a relative turn
    #     "speed":     0.4,
    #     "tolerance_deg": 2,
    #     "timeout":   200,
    # },

    # {
    #     "type":     "line_follow",
    #     "side":     "right",
    #     "speed":    0.4,
    #     "timeout":  30
    # },
    

    # # line follow until x,y (5.41, -2.00) 3rd straight after roundabout
    # {
    #     "type":     "line_follow_until_xy",
    #     "side":     "right",
    #     "speed":    0.3,
    #     "target_x": 5.41,
    #     "target_y": -2.33,
    #     "distance_tolerance": 0.1,
    #     "timeout":  40
    # },

    # # drive to extra time point 1 
    # {
    #     "type":     "drive_to_xy",
    #     "target_x": 3.71,
    #     "target_y": -2.33,
    #     "speed":    0.3,
    #     "timeout":  30
    # },

    # {
    #     "type":     "drive_to_xy", 
    #     "target_x": 5.410,
    #     "target_y": -2.33,
    #     "speed":    0.3,
    #     "timeout":  30
    # },

    # {
    #     "type":     "line_follow",
    #     "side":     "right",
    #     "speed":    0.4,
    #     "timeout":  30
    # },

]