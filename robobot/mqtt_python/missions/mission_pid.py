
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
        "target_yaw": 0,
        "max_speed":  0.5,
        "timeout":  30
    },

    {
        "type":      "turn_to_heading_iwo",
        "heading_deg": 90, # 0 is the heading at the beginning ot the mission, so this is a relative turn
        "speed":     1.2,
        "tolerance_deg": 2,
        "timeout":   200,
    },

    {
        "type":     "drive_to_xyyaw_IWO_fast",
        "target_x": 1,
        "target_y": 1,
        "target_yaw": 90,
        "max_speed":  0.5,
        "timeout":  30
    },

    {
        "type":      "turn_to_heading_iwo",
        "heading_deg": 180, # 0 is the heading at the beginning ot the mission, so this is a relative turn
        "speed":     1.2,
        "tolerance_deg": 2,
        "timeout":   200,
    },

    {
        "type":     "drive_to_xyyaw_IWO_fast",
        "target_x": 0,
        "target_y": 1,
        "target_yaw": 180,
        "max_speed":  0.5,
        "timeout":  30
    },

    {
        "type":      "turn_to_heading_iwo",
        "heading_deg": 270, # 0 is the heading at the beginning ot the mission, so this is a relative turn
        "speed":     1.2,
        "tolerance_deg": 2,
        "timeout":   200,
    },

    {
        "type":     "drive_to_xyyaw_IWO_fast",
        "target_x": 0,
        "target_y": 0,
        "target_yaw": 270,
        "max_speed":  0.5,
        "timeout":  30
    },

    {
        "type":      "turn_to_heading_iwo",
        "heading_deg": 360, # 0 is the heading at the beginning ot the mission, so this is a relative turn
        "speed":     1.2,
        "tolerance_deg": 2,
        "timeout":   200,
    },


    # {
    #     "type":      "turn_to_heading_iwo",
    #     "heading_deg": 90, # 0 is the heading at the beginning ot the mission, so this is a relative turn
    #     "speed":     1.2,
    #     "tolerance_deg": 2,
    #     "timeout":   200,
    # },



    # {
    #     "type":      "turn_to_heading_iwo",
    #     "heading_deg": 90, # 0 is the heading at the beginning ot the mission, so this is a relative turn
    #     "speed":     1.2,
    #     "tolerance_deg": 2,
    #     "timeout":   200,
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
    #     "nominal_speed":    1, # 1
    #     "stop_speed": 1, #1
    #     "turn_rate": 0.6,
    #     "final_turn_rate": 0,
    #     "heading_deg": -10,
    #     "timeout":  200,
    #     "tolerance": 2,
    # },



    
    # {
    #     "type":  "reset_IWO",
    #     "x": 6.30,
    #     "y": 1.72+0.84, # 1.72
    #     "z": 0.47,
    #     "roll":    0,
    #     "pitch":  0,
    #     "yaw":  90
    # },
    # {
    #     "type":     "drive_to_xyyaw_IWO_fast",
    #     "target_x": 6.3, #
    #     "target_y": 1.72+0.84+0.3,
    #     "target_yaw": 190,
    #     "max_speed":    0.2, #0.5
    #     "timeout":  30
    # },
    # {
    # "type": "drive_to_xyyaw_IWO",
    # "target_yaw": 190, 
    # "just_yaw": True
    # },

    
  
    # {
    #     "type": "drive_to_xyyaw_IWO",
    #     "target_yaw": 160, #145
    #     "just_yaw": True
    # },
    
    {
        "type": "servo_down"
    },

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