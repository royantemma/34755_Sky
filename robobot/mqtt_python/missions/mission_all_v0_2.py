"""
missions/mission_all_v0_0.py
---------------------
version 0.2, try to add eight

Route: Start -> line follow -> take left at junction -> go on roundabout -> use vision to go around roundabout -> exit roundabout North using heading and update position 
-> line follow until x,y coord (5410,-2000) using odometry -> go to extra time point 1 (in mm and robo coord (3309,-2364))-> return to line and continue line following up the ramp and back down
-> go to coord (290,-3583) -> back up to (1550,-3583) -> rotate left 90 deg -> drive forward to (1550,-409) -> line follow -> Goal
"""

TOTAL_TIME       = 120
GOAL_TIME_BUFFER = 20

TASKS = [

    # {
    #     "type": "line_follow_vision", # controlling speed ? stops ? crossings ?
    #     "timeout": 20,
    #     "junction": 'left',
    #     "start_speed": 0,
    #     "nominal_speed": 0.6,
    #     "stop_speed": 0.25,
    #     "turn_sensitivity": 0.005,
    #     "time_to_full_speed": 0.2,
    #     "stop_time": 0.4,
    #     "min_pixels_to_detect_line": 200
    # }, 

    # {
    #     "type":     "drive_straight",
    #     "speed":    0.25,
    #     "distance": 0.15,
    #     "timeout":  200,
    #     "is_stopping_at_end": True
    # },

    # {
    #     "type":     "drive_curved",
    #     "nominal_speed":    0.25,
    #     "stop_speed": 0.25,
    #     "turn_rate": 1,
    #     "final_turn_rate": 0,
    #     "heading_deg": -15,
    #     "timeout":  200,
    #     "tolerance": 2,
    # },

    # # {
    # #     "type":     "drive_roundabout",
    # #     "nominal_speed":    0.4,
    # #     "stop_speed": 0.4,
    # #     "turn_rate": 1.15,
    # #     "final_turn_rate": 0,
    # #     "heading_deg": 15,
    # #     "timeout":  200,
    # #     "tolerance": 2,
    # #     "min_run_time": 2.5
    # # },


    # { # Theo: I tried higher speeds, but I didn't managed to get it work well, so don't loose your time on this ;)
    #     "type": "roundabout_vision", # controlling speed ? stops ?
    #     "timeout": 10,
    #     "speed": 0.4, # 0.4 recommended
    #     "start_speed": 0.25, # choose same as before
    #     "stop_speed": 0.4,
    #     "time_to_full_speed": 0.06,
    #     "Kp": 6, # 6 recommended (not implemented)
    #     "Ki": 0.5, # 0.2 recommended (not implemented)
    #     "percentage_of_white": 30 # this parameters tunes the center of the roundabout (i think smaller is smaller radius)
    # },

    # {
    #     "type":     "drive_straight",
    #     "speed":    0.4,
    #     "distance": 0.2,
    #     "timeout":  200,
    #     "is_stopping_at_end": False
    # },

    # {
    #     "type": "line_follow_vision",
    #     "timeout": 8,
    #     "junction": 'right',
    #     "start_speed": 0.4, # choose same as speed in "roundabout_vision"
    #     "nominal_speed": 1, # nice value is 0.6
    #     "stop_speed": 0,
    #     "turn_sensitivity": 0.007, # nice value is 0.005
    #     "time_to_full_speed": 2.5,
    #     "stop_time": 1.5
    # }, 

    {
        "type": "wait_for_police",
        "percentage_height": 70,
        'time_between_checks': 0.1,
        "min_distance_for_movement": 5,
        "min_free_frames_to_move": 5,
        "timeout": 20
    },

    {
        "type":     "drive_straight",
        "speed":    0.3,
        "distance": 0.8,
        "timeout":  200,
        "is_stopping_at_end": True
    },

    {
        "type":      "turn_to_heading",
        "heading_deg": -70, # 0 is the heading at the beginning ot the mission, so this is a relative turn
        "speed":     0.4,
        "tolerance_deg": 2,
        "timeout":   200,
    },

    {
        "type": "wait_for_police",
        "percentage_height": 75,
        "time_between_checks": 0.2,
        "min_distance_for_movement": 1,
        "min_free_frames_to_move": 7,
        "timeout": 20
    },

    # {
    #     "type":     "drive_straight",
    #     "speed":    0.4,
    #     "distance": 0.5,
    #     "timeout":  200,
    #     "is_stopping_at_end": True
    # },

    # {
    #     "type":     "drive_curved",
    #     "nominal_speed":    0.4,
    #     "stop_speed": 0.4,
    #     "turn_rate": 0.5,
    #     "final_turn_rate": 0,
    #     "heading_deg": -60,
    #     "timeout":  200,
    #     "tolerance": 2,
    # },

    {
        "type": "line_follow_vision",
        "timeout": 1,
        "junction": 'straight',
        "start_speed": 0.4, # choose same as speed in "roundabout_vision"
        "nominal_speed": 0.4, # nice value is 0.6
        "stop_speed": 0.4,
        "turn_sensitivity": 0.005, # nice value is 0.005
        "time_to_full_speed": 0.4,
        "stop_time": 1
    }, 


    {
        "type": "line_follow_vision",
        "timeout": 5,
        "junction": 'left',
        "start_speed": 0.4, # choose same as speed in "roundabout_vision"
        "nominal_speed": 0.4, # nice value is 0.6
        "stop_speed": 0,
        "turn_sensitivity": 0.006, # nice value is 0.005
        "time_to_full_speed": 0.4,
        "stop_time": 1.5,
        "percentage_height": 10,
        "final_heading": [-185, -175] # first smallest (or most negative) element
    }, 

    {
        "type":     "drive_straight",
        "speed":    0.3,
        "distance": 0.8,
        "timeout":  200,
        "is_stopping_at_end": True
    },



    # {
    #     "type":     "drive_straight",
    #     "speed":    0.01,
    #     "distance": 0.2,
    #     "timeout":  200,
    #     "is_stopping_at_end": False
    # },

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