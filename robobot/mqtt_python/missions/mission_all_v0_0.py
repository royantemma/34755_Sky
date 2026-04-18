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
        "type":     "line_follow",
        "side":     "left",
        "speed":    0.4,
        "timeout":  30
    },

    {
        "type":     "drive_straight",
        "speed":    0.3,
        "distance": 0.35,
        "timeout":  200,
    },

    {
        "type":      "turn_to_heading",
        "heading_deg": -18, # 0 is the heading at the beginning ot the mission, so this is a relative turn
        "speed":     0.4,
        "tolerance_deg": 2,
        "timeout":   200,
    },

    {
        "type": "roundabout_vision", # controlling speed ? stops ?
        "timeout": 15
    },

        {
        "type": "line_follow_vision", # controlling speed ? stops ? crossings ? brakes ?
        "timeout": 20
    },
    
    {
        "type": "roundabout_vision", # controlling speed ? stops ?
        "timeout": 15
    },

    {
        "type": "line_follow_vision", # controlling speed ? stops ? crossings ?
        "timeout": 20
    },

]