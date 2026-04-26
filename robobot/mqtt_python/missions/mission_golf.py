
    # 1. golf ball on seesaw

    # add left junction line follow task to get on seesaw

    # approach pos on seesaw

TOTAL_TIME       = 200
GOAL_TIME_BUFFER = 20

TASKS = [
    {
        "type":  "servo_down",
    },
    {
        "type":  "servo_relax",
    },
    {
    "type":     "reset_IWO",
    "x": 3.05,
    "y": 1.72,
    "z": 0,
    "roll":    0,
    "pitch":  0,
    "yaw":  90
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
# Drive onto ramp
{
    "type": "linefollow_until_ramp_IWO",
    "speed": 0.4, # 0.4 seems robust
},
{
    "type": "correct_IWO",
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
    "type": "drive_straight_crossing_x_lines",
    "speed": 0.4, # 0.4 seems robust
    "lines_to_cross": 1,
},

#{
#    "type": "linefollow_until_intersection",
#    "side": "left",
#    "timeout": 30,
#    "speed": 0.4,
#},




# Get to stairs


]
"""
{
    "type": "servo_up",
},
{
    "type":     "reset_IWO",
    "x": 6.30,
    "y": -1.20,
    "z": 0,
    "roll":    0,
    "pitch":  0,
    "yaw":  90
},
# Stop at intersection before seesaw
{
    "type": "linefollow_until_intersection",
    "side": "left",
    "timeout": 30,
    "speed": 0.6,
},
{
    "type":  "correct_IWO",
    "x": 6.30,
    "y": 1.72,
    "z": 0.47,
    "roll":    0,
    "pitch":  0,
    "yaw":  90
},
#{
#    "type":  "turn_to_heading_iwo",
#    "heading_deg": 180,
#},
# Drive onto seesaw

{
    "type":     "drive_to_xyyaw_IWO",
    "target_x": 5.6,# 5.50,
    "target_y": 1.72-0.05,
    "target_yaw": 185,
    "max_speed":    0.4,
    "timeout":  30
},
# Follow line on seesaw 
{
    "type": "line_follow_until_xy_IWO",
    "side": "left",
    "timeout": 30,
    "target_x": 5.1,
    "target_y": 1.72,
    "speed": 0.3,
},
{
    "type":  "get_ball",
},
{
    "type":  "line_follow",
    "side": "left",
    "timeout": 30,
    "speed": 0.3,
},
{
    "type":  "servo_relax",
},


{
    "type":     "drive_to_xyyaw_IWO",
    "target_x": 3.05,
    "target_y": 1.72,
    "target_yaw": 90,
    "max_speed":    0.2,
    "timeout":  30
},
#{
#    "type":  "go_to",
#    "heading_deg": 180,
#},
"""



"""
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
# maybe add iwo corrct here
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
"""
"""

"""
#{
    #   "type":  "servo_up",
#},