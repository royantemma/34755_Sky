
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

#     {
#         "type": "line_follow_vision", # controlling speed ? stops ? crossings ?
#         "timeout": 20,
#         "junction": 'left',
#         "start_speed": 0,
#         "nominal_speed": 0.6,
#         "stop_speed": 0.25,
#         "turn_sensitivity": 0.005,
#         "time_to_full_speed": 0.2,
#         "stop_time": 0.4,
#         "min_pixels_to_detect_line": 200
#     }, 

#     {
#         "type":     "drive_straight",
#         "start_speed": 0.25,
#         "speed":    0.25,
#         "distance": 0.15,
#         "timeout":  200,
#         "is_stopping_at_end": True
#     },

#     {
#         "type":     "drive_curved",
#         "nominal_speed":    0.25,
#         "stop_speed": 0.25,
#         "turn_rate": 1,
#         "final_turn_rate": 0,
#         "heading_deg": -15,
#         "timeout":  200,
#         "tolerance": 2,
#     },



#     { # Theo: I tried higher speeds, but I didn't managed to get it work well, so don't loose your time on this ;)
#         "type": "roundabout_vision", # controlling speed ? stops ?
#         "timeout": 10,
#         "speed": 0.4, # 0.4 recommended
#         "start_speed": 0.25, # choose same as before
#         "stop_speed": 0.4,
#         "time_to_full_speed": 0.06,
#         "Kp": 12, # 6 recommended (not implemented)
#         "Ki": 0, # 0.2 recommended (not implemented)
#         "percentage_of_white": 60 # this parameters tunes the center of the roundabout (i think bigger is smaller radius)
#     },

#     {
#         "type":     "drive_straight",
#         "start_speed": 0.4,
#         "speed":    0.4,
#         "distance": 0.2,
#         "timeout":  200,
#         "is_stopping_at_end": False
#     },

#     # {
#     #     "type": "line_follow_vision",
#     #     "timeout": 7,
#     #     "junction": 'right',
#     #     "start_speed": 0.4, # choose same as speed in "roundabout_vision"
#     #     "nominal_speed": 0.9, # nice value is 0.6
#     #     "stop_speed": 0,
#     #     "turn_sensitivity": 0.007, # nice value is 0.005
#     #     "time_to_full_speed": 2,
#     #     "stop_time": 1
#     # }, 

# {
#         "type": "line_follow_vision",
#         "timeout": 15,
#         "junction": 'straight',
#         "start_speed": 0.4, 
#         "nominal_speed": 0.8, # nice value is 0.6
#         "stop_speed": 0.4,
#         "turn_sensitivity": 0.007, # nice value is 0.005
#         "time_to_full_speed": 2,
#         "stop_time": 0.1,
#          "final_heading": [80, 100]
#     }, 




#     {
#     "type": "linefollow_until_x_intersections",
#     "side": "left",
#     "timeout": 30,
#     "speed": 0.5,
#     },
    
#     {
#         "type":  "reset_IWO",
#         "x": 6.30,
#         "y": 1.72, # 1.72
#         "z": 0.47,
#         "roll":    0,
#         "pitch":  0,
#         "yaw":  90
#     },
#     {
#         "type": "drive_to_xyyaw_IWO",
#         "target_yaw": 180,
#         "target_x": 6.30,
#         "target_y": 1.74,
#     },


#     {
#         "type": "line_follow_vision", # controlling speed ? stops ? crossings ?
#         "timeout": 5,
#         "junction": 'straight',
#         "start_speed": 0,
#         "nominal_speed": 0.2,
#         "stop_speed": 0,
#         "turn_sensitivity": 0.005,
#         "time_to_full_speed": 0.2,
#         "stop_time": 0.1,
#     }, 


# {
#      "type":  "get_ball",
# },

# # {
# #         "type":             "line_follow_brake",
# #         "side":             "right",
# #         "speed":            0.3,
# #         "brake_speed":      -0.05,
# #         "max_speed":        0.35,
# #         "timeout":          200,
# #         "recovery":         True,
# #         "recovery_time":    0.1,
    
    
# #     },

# {
#      "type":  "line_follow",
#      "side": "left",
#      "timeout": 7,
#      "speed": 0.3,
# },

# # New edit
# {
#     "type": "servo_relax",
# },

# {
#     "type": "drive_to_xyyaw_IWO",
#     "target_yaw": 160, # 
#     "just_yaw": True
# },

# {
#     "type": "drive_until_line_detection",
#     "speed": 0.4,
#     "timeout": 30
# },
# {
#     "type": "drive_to_xyyaw_IWO",
#     "target_yaw": 90, # 0 is good for small speeds, but -5 is for when the robot overshoots
#     "just_yaw": True
# },
#     {
#     "type": "linefollow_until_x_intersections",
#     "side": "left",
#     "timeout": 30,
#     "speed": 0.4,
#     "stop_speed": 0, # too not fall back
#     "intersection_count": 2
#     },

#     {
#         "type": "drive_to_xyyaw_IWO",
#         "target_yaw": -15, # 0 is good for small speeds, but -5 is for when the robot overshoots
#         "just_yaw": True
#     },


# # {
# #     "type":     "drive_to_xyyaw_IWO",
# #     "target_x": 3.5,
# #     "target_y": 1.72,
# #     "target_yaw": 90,
# # },
# # Drive to ramp line
# # {
# #     "type": "drive_straight_crossing_x_lines",
# #     "speed": 0.4, # 0.4 seems robust
# #     "lines_to_cross": 2,
# # },
# # # Turn onto line
# # {
# #     "type": "drive_to_xyyaw_IWO",
# #     "target_yaw": 0, # 0 is good for small speeds, but -5 is for when the robot overshoots
# #     "just_yaw": True
# # },


# Relax servo to avoid breaking it on the ramp
{
    "type":  "servo_relax",
},
# Drive onto ramp

# {
#     "type": "linefollow_until_ramp_IWO",
#     "speed": 0.4, # 0.4 seems robust
# },

    {
        "type": "line_follow_vision", # controlling speed ? stops ? crossings ?
        "timeout": 25,
        "junction": 'straight',
        "start_speed": 0,
        "nominal_speed": 0.2,
        "stop_speed": 0,
        "turn_sensitivity": 0.005,
        "time_to_full_speed": 0.2,
        #"stop_time": 0.001,
        "time_before_ramp_heading": 10
    }, 

    {
        "type": "turn_to_heading",
        "heading_deg": 0,
        "speed": 0.2,
        "tolerance": 2,
        "timeout": 5
    },

    {
        "type":     "drive_straight",
        "start_speed": 0.2,
        "speed":    0.2,
        "distance": 0.2,
        "timeout":  200,
        "is_stopping_at_end": True
    },

{
    "type": "reset_IWO",
    "x": 5.6+0.1, # 5.6+0.2
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
    "target_yaw": 145, 
    "just_yaw": True
},
### GOOO HOME
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


# #######
# {
#     "type": "drive_to_xyyaw_IWO",
#     "target_yaw": -90, 
#     "just_yaw": True
# },
# {
#     "type": "drive_until_line_detection",
#     "speed": 0.2, #
# },
# {
#     "type": "drive_to_xyyaw_IWO",
#     "target_yaw": -180, 
#     "just_yaw": True
# },
# {
#     "type": "servo_mid"
# },
# {
#         "type": "stairs_down",
#         "side": "left",                 # Follow left edge of line
#         "fast_speed": 0.4,              # Speed when horizontal (m/s) - reduced for stability
#         "slow_speed": 0.12,             # Speed when descending (m/s) - reduced for gentler control
#         "step_distance": 0.08,          # Distance per stair (meters) - reduced for better control
#         "pitch_threshold": 5,        # Pitch ITS DEG
#         "num_stairs": 5,                # Number of stairs to descend
#         "timeout": 60,                  # Timeout for entire task (seconds)
#     },
    
#     # Servo up to deactivate line sensor
#     {
#         "type": "servo_up",
#     },

# # {
# #     "type":  "servo_relax",
# # },


# # {
# #     "type":     "drive_to_xyyaw_IWO",
# #     "target_x": 3.05,
# #     "target_y": 1.72,
# #     "target_yaw": 90,
# #     "max_speed":    0.2,
# #     "timeout":  30
# # },


# #     #{
# #     #    "type":     "drive_to_xyyaw_IWO_fast",
# #     #    "target_x": 4.95,
# #     #    "target_y": -1.5,
# #     #    "target_yaw": 155,
# #     #    "max_speed":    0.8, #0.5
# #     #    "timeout":  30
# #     #},


# #     #{"x": 4.95,
# #     #    "y": -1.2,
# #     #    "type": "line_follow_vision",
# #     #    "timeout": 2,
# #     #    "junction": 'straight',
# #     #    "start_speed": 1, 
# #     #   "nominal_speed": 1, # nice value is 0.6
# #     #    "stop_speed": 0,
# #     #    "turn_sensitivity": 0.007, # nice value is 0.005
# #     #    "time_to_full_speed": 0.3,
# #     #    "stop_time": 0.6
# #     #}, 


]   
