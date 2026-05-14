
TOTAL_TIME       = 120
GOAL_TIME_BUFFER = 20

TASKS = [

    {
        "type": "servo_up",
    },


    {
        "type":  "reset_IWO",
        "x": 0,
        "y": 0, 
        "z": 0,
        "roll":    0,
        "pitch":  0,
        "yaw":  0
    },


    {
        "type":     "drive_to_xyyaw_IWO_fast",
        "target_x": 1,
        "target_y": 0,
        "target_yaw": 90,
        "max_speed":  0.5,
        "timeout":  30
    },

    {
        "type":     "drive_to_xyyaw_IWO_fast",
        "target_x": 1,
        "target_y": 1,
        "target_yaw": 90+90,
        "max_speed":  0.5,
        "timeout":  30
    },

    {
        "type":     "drive_to_xyyaw_IWO_fast",
        "target_x": 0,
        "target_y": 1,
        "target_yaw": 90+90+90,
        "max_speed":  0.5,
        "timeout":  30
    },

    {
        "type":     "drive_to_xyyaw_IWO_fast",
        "target_x": 0,
        "target_y": 0,
        "target_yaw": 90+90+90+90,
        "max_speed":  0.5,
        "timeout":  30
    },

    # {
    #     "type":      "turn_to_heading_iwo",
    #     "heading_deg": 90, # 0 is the heading at the beginning ot the mission, so this is a relative turn
    #     "speed":     1.2,
    #     "tolerance_deg": 2,
    #     "timeout":   200,
    # },

    # {
    #     "type":     "drive_to_xyyaw_IWO_fast",
    #     "target_x": 1,
    #     "target_y": 1,
    #     "target_yaw": 90,
    #     "max_speed":  0.5,
    #     "timeout":  30
    # },

    # {
    #     "type":      "turn_to_heading_iwo",
    #     "heading_deg": 180, # 0 is the heading at the beginning ot the mission, so this is a relative turn
    #     "speed":     1.2,
    #     "tolerance_deg": 2,
    #     "timeout":   200,
    # },

    # {
    #     "type":     "drive_to_xyyaw_IWO_fast",
    #     "target_x": 0,
    #     "target_y": 1,
    #     "target_yaw": 180,
    #     "max_speed":  0.5,
    #     "timeout":  30
    # },

    # {
    #     "type":      "turn_to_heading_iwo",
    #     "heading_deg": 270, # 0 is the heading at the beginning ot the mission, so this is a relative turn
    #     "speed":     1.2,
    #     "tolerance_deg": 2,
    #     "timeout":   200,
    # },

    # {
    #     "type":     "drive_to_xyyaw_IWO_fast",
    #     "target_x": 0,
    #     "target_y": 0,
    #     "target_yaw": 270,
    #     "max_speed":  0.5,
    #     "timeout":  30
    # },

    # {
    #     "type":      "turn_to_heading_iwo",
    #     "heading_deg": 360, # 0 is the heading at the beginning ot the mission, so this is a relative turn
    #     "speed":     1.2,
    #     "tolerance_deg": 2,
    #     "timeout":   200,
    # },


    # {
    #     "type": "servo_down"
    # },

    # {
    #     "type":     "correct_IWO",
    #     "x": 4.95,
    #     "y": -1.2,
    #     "z": 0,
    #     "roll":    0,
    #     "pitch":  0,
    #     "yaw":  -90
    # },

]