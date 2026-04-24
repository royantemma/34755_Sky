
    # 1. golf ball on seesaw

    # add left junction line follow task to get on seesaw

    # approach pos on seesaw

TOTAL_TIME       = 200
GOAL_TIME_BUFFER = 20

TASKS = [
    {
        "type":     "reset_IWO",
        "x": 6.30,
        "y": -1.20,
        "z": 0,
        "roll":    0,
        "pitch":  0,
        "yaw":  90
    },
    # Stop at intersection befor seesaw
    {
        "type": "linefollow_until_intersection",
        "side": "left",
        "timeout": 30,
        "speed": 0.8,
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
        "target_x": 5.25,
        "target_y": 1.72,
        "target_yaw": 180,
        "max_speed":    0.4,
        "timeout":  30
    },
    {
        "type":  "servo_down",
    },
    {
        "type":  "servo_up",
    },
    {
        "type":  "get_ball",
    },
    #{
    #    "type":  "go_to",
    #    "heading_deg": 180,
    #},

    
]
