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
        {
        "type": "line_follow_vision", # controlling speed ? stops ? crossings ?
        "timeout": 20,
        "junction": 'left',
        "start_speed": 0,
        "nominal_speed": 0.6,
        "stop_speed": 0.25,
        "turn_sensitivity": 0.005,
        "time_to_full_speed": 0.2,
        "stop_time": 0.4,
        "min_pixels_to_detect_line": 200
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
        "type":     "drive_curved",
        "nominal_speed":    0.25,
        "stop_speed": 0.25,
        "turn_rate": 1,
        "final_turn_rate": 0,
        "heading_deg": -15,
        "timeout":  200,
        "tolerance": 2,
    },



    { # Theo: I tried higher speeds, but I didn't managed to get it work well, so don't loose your time on this ;)
        "type": "roundabout_vision", # controlling speed ? stops ?
        "timeout": 10,
        "speed": 0.4, # 0.4 recommended
        "start_speed": 0.25, # choose same as before
        "stop_speed": 0.4,
        "time_to_full_speed": 0.06,
        "Kp": 12, # 6 recommended (not implemented)
        "Ki": 0, # 0.2 recommended (not implemented)
        "percentage_of_white": 60 # this parameters tunes the center of the roundabout (i think bigger is smaller radius)
    },

    {
        "type":     "drive_straight",
        "start_speed": 0.4,
        "speed":    0.4,
        "distance": 0.2,
        "timeout":  200,
        "is_stopping_at_end": False
    },

    {
        "type": "line_follow_vision",
        "timeout": 7,
        "junction": 'straight',
        "start_speed": 0.4, # choose same as speed in "roundabout_vision"
        "nominal_speed": 0.9, # nice value is 0.6
        "stop_speed": 0,
        "turn_sensitivity": 0.007, # nice value is 0.005
        "time_to_full_speed": 2,
        "stop_time": 1,
        "time_after_turn": 1.55
    }, 
    


    
    {
       "type":     "reset_IWO",
       "x": 4.95,
        "y": -1.2,
        "z": 0,
       "roll":    0,
        "pitch":  0,
        "yaw":  -90
    }, 

    {
        "type":     "drive_to_xyyaw_IWO_fast",
        "target_x": 3.55,
        "target_y": -1.85,
        "target_yaw": 155,
        "max_speed":    0.8, #0.5
        "timeout":  30
    },

    {
        "type": "wait_for_police",
        "percentage_height": 75,
        'time_between_checks': 0.2, #0.15
        "min_distance_for_movement": 5,
        "min_free_frames_to_move": 7, #5
        "timeout": 20,
        "require_movement_before_start": True
    },

    {
        "type":     "drive_curved_distance",
        "speed":    0.5,
        "distance": 0.8,
        "turn_rate": -0.05, # -0.23   "timeout":  200,
        "is_stopping_at_end": True
    },



    {
        "type":      "turn_to_heading_iwo",
        "heading_deg": 90, # 0 is the heading at the beginning ot the mission, so this is a relative turn
        "speed":     1.2,
        "tolerance_deg": 2,
        "timeout":   200,
    },

    {
        "type": "wait_for_police",
        "percentage_height": 75,
        "time_between_checks": 0.2, # originally 0.2
        "min_distance_for_movement": 1,
        "min_free_frames_to_move": 7,
        "timeout": 20,
        "require_movement_before_start": False,
        "percentage_from_bottom": 20
    },


    ####


    {
        "type": "line_follow_vision",
        "timeout": 1,
        "junction": 'left',
        "start_speed": 0, 
        "nominal_speed": 0.4, # nice value is 0.6
        "stop_speed": 0.4,
        "turn_sensitivity": 0.005, # nice value is 0.005
        "time_to_full_speed": 0.3,
        "percentage_height": 35,
        "stop_time": 0.4
    }, 


    {
        "type": "line_follow_vision",
        "timeout": 7,
        "junction": 'left',
        "start_speed": 0.4, # choose same as speed in "roundabout_vision"
        "nominal_speed": 0.4, # nice value is 0.6
        "stop_speed": 0.4,
        "turn_sensitivity": 0.006, # nice value is 0.005
        "time_to_full_speed": 0.4,
        "stop_time": 1.5,
        "percentage_height": 10,
        "final_heading": [0, 20] # first smallest (or most negative) element
    }, 

    {
        "type":     "drive_straight",
        "start_speed": 0.4,
        "speed":    0.8,
        "distance": 0.55,
        "timeout":  200,
        "is_stopping_at_end": False,
        "time_to_full_speed": 0.3
    },

    {
        "type":     "drive_curved_iwo",
        "nominal_speed":    1,
        "start_speed": 0.8,
        "stop_speed": 1,
        "turn_rate": 1.5,
        "final_turn_rate": 0,
        "heading_deg": -70,
        "timeout":  200,
        "tolerance": 2,
        "time_to_full_speed": 0.3
    },
    {
        "type":     "drive_straight",
        "start_speed": 1,
        "speed":    1,
        "distance": 0.2,
        "timeout":  200,
        "is_stopping_at_end": False,
        "time_to_full_speed": 0.3
    },

    {
        "type":     "drive_curved_iwo",
        "nominal_speed":    1,
        "stop_speed": 1,
        "turn_rate": 0.65,
        "final_turn_rate": 0,
        "heading_deg": -10,
        "timeout":  200,
        "tolerance": 2,
    },


    {
        "type": "line_follow_vision",
        "timeout": 5,
        "junction": 'straight',
        "start_speed": 1, 
        "nominal_speed": 0.6, # nice value is 0.6
        "stop_speed": 0.4,
        "turn_sensitivity": 0.007, # nice value is 0.005
        "time_to_full_speed": 0.3,
        "stop_time": 0.1,
         "final_heading": [80, 100]
    }, 

    # {
    #     "type":     "drive_straight",
    #     "start_speed": 0.4,
    #     "speed":    0.8,
    #     "distance": 0.55,
    #     "timeout":  200,
    #     "is_stopping_at_end": False,
    #     "time_to_full_speed": 0.3
    # },

    # {
    #     "type":     "drive_curved_iwo",
    #     "nominal_speed":    1,
    #     "start_speed": 0.8,
    #     "stop_speed": 1,
    #     "turn_rate": 1.5,
    #     "final_turn_rate": 0,
    #     "heading_deg": -70,
    #     "timeout":  200,
    #     "tolerance": 2,
    #     "time_to_full_speed": 0.3
    # },
    

    # {
    #     "type":     "drive_curved_iwo",
    #     "nominal_speed":    0.8,
    #     "stop_speed": 0.4,
    #     "turn_rate": 0.5,
    #     "final_turn_rate": 0,
    #     "heading_deg": -10,
    #     "timeout":  200,
    #     "tolerance": 2,
    # },
    # {
    # "type": "drive_until_line_detection",
    # "speed": 0.4, #
    # },
    # {
    # "type": "drive_to_xyyaw_IWO",
    # "target_yaw": 90, 
    # "just_yaw": True
    # },
    {
    "type": "linefollow_until_intersection",
    "side": "left",
    "timeout": 30,
    "speed": 0.6,
},
{
    "type":  "reset_IWO",
    "x": 6.30,
    "y": 1.72,
    "z": 0.47,
    "roll":    0,
    "pitch":  0,
    "yaw":  90
},
{
    "type": "drive_to_xyyaw_IWO",
    "target_yaw": 180,
    "just_yaw": True
},
# {
#     "type":  "turn_to_heading_iwo",
#     "heading_deg": 180,
# },
# Drive onto seesaw
# {
#         "type":     "drive_straight",
#         "start_speed": 0,
#         "speed":    0.2,
#         "distance": 0.3,
#         "timeout":  200,
#         "is_stopping_at_end": False,
#         "time_to_full_speed": 0.1
#     },

    {
        "type": "line_follow_vision", # controlling speed ? stops ? crossings ?
        "timeout": 4,
        "junction": 'straight',
        "start_speed": 0,
        "nominal_speed": 0.2,
        "stop_speed": 0,
        "turn_sensitivity": 0.005,
        "time_to_full_speed": 0.2,
        "stop_time": 0.1,
    }, 
# {
#     "type":     "drive_to_xyyaw_IWO",
#     "target_x": 5.6,# 5.50,
#     "target_y": 1.72-0.05,
#     "target_yaw": 185,
#     "max_speed":    0.4,
#     "timeout":  30
#},
# Follow line on seesaw 

{
     "type":  "get_ball",
},
{
     "type":  "line_follow",
     "side": "left",
     "timeout": 30,
     "speed": 0.3,
},

# New edit
{
    "type": "servo_relax",
},


{
    "type":     "drive_to_xyyaw_IWO",
    "target_x": 3.5,
    "target_y": 1.72,
    "target_yaw": 90,
},
# Drive to ramp line
{
    "type": "drive_straight_crossing_x_lines",
    "speed": 0.4, # 0.4 seems robust
    "lines_to_cross": 2,
},
# Turn onto line
{
    "type": "drive_to_xyyaw_IWO",
    "target_yaw": 0, # 0 is good for small speeds, but -5 is for when the robot overshoots
    "just_yaw": True
},
# Relax servo to avoid breaking it on the ramp
{
    "type":  "servo_relax",
},
# Drive onto ramp
{
    "type": "linefollow_until_ramp_IWO",
    "speed": 0.4, # 0.4 seems robust
},
{
    "type": "reset_IWO",
    "x": 5.6+0.2,
    "y": 3.46,
    "z": 0.55,
    "roll":    0,
    "pitch":  0,
    "yaw":  0
},
# put ball in the hole
{
    "type":  "put_ball_in_hole",
    "hole_x": 6.4,
    "hole_y": 3.57 -0.05, # added MINUS
},
{
    "type": "drive_to_xyyaw_IWO",
    "target_yaw": 45, 
    "just_yaw": True
},
{
    "type": "drive_until_line_detection",
    "speed": -0.2,
    "timeout": 30
},
# Turn to new ball
{
    "type": "drive_to_xyyaw_IWO",
    "target_yaw": -90, 
    "just_yaw": True
},
{
    "type": "get_ball",
},
{
    "type": "drive_straight_crossing_x_lines",
    "speed": -0.2, 
    "lines_to_cross": 2,
},
# put ball in the hole 
{
    "type":  "put_ball_in_hole",
    "hole_x": 6.4,
    "hole_y": 3.57,
},
{
    "type": "drive_to_xyyaw_IWO",
    "target_yaw": 45, 
    "just_yaw": True
},
{
    "type": "drive_until_line_detection",
    "speed": -0.2,
    "timeout": 30
},
{
    "type": "drive_to_xyyaw_IWO",
    "target_yaw": -90, 
    "just_yaw": True
},
{
    "type": "drive_until_line_detection",
    "speed": 0.2, #
},
{
    "type": "drive_to_xyyaw_IWO",
    "target_yaw": -180, 
    "just_yaw": True
},
{
    "type": "servo_mid"
},
{
        "type": "stairs_down",
        "side": "left",                 # Follow left edge of line
        "fast_speed": 0.4,              # Speed when horizontal (m/s) - reduced for stability
        "slow_speed": 0.12,             # Speed when descending (m/s) - reduced for gentler control
        "step_distance": 0.08,          # Distance per stair (meters) - reduced for better control
        "pitch_threshold": 5,        # Pitch ITS DEG
        "num_stairs": 5,                # Number of stairs to descend
        "timeout": 60,                  # Timeout for entire task (seconds)
    },
    
    # Servo up to deactivate line sensor
    {
        "type": "servo_up",
    },

# {
#     "type":  "servo_relax",
# },


# {
#     "type":     "drive_to_xyyaw_IWO",
#     "target_x": 3.05,
#     "target_y": 1.72,
#     "target_yaw": 90,
#     "max_speed":    0.2,
#     "timeout":  30
# },


#     #{
#     #    "type":     "drive_to_xyyaw_IWO_fast",
#     #    "target_x": 4.95,
#     #    "target_y": -1.5,
#     #    "target_yaw": 155,
#     #    "max_speed":    0.8, #0.5
#     #    "timeout":  30
#     #},


#     #{"x": 4.95,
#     #    "y": -1.2,
#     #    "type": "line_follow_vision",
#     #    "timeout": 2,
#     #    "junction": 'straight',
#     #    "start_speed": 1, 
#     #   "nominal_speed": 1, # nice value is 0.6
#     #    "stop_speed": 0,
#     #    "turn_sensitivity": 0.007, # nice value is 0.005
#     #    "time_to_full_speed": 0.3,
#     #    "stop_time": 0.6
#     #}, 


]   
"""


    
"""    