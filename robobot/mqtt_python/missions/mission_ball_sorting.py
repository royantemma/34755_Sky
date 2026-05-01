TOTAL_TIME       = 120
GOAL_TIME_BUFFER = 20

TASKS = [
        # 1. Reset Odometry at the starting point of the Luggage Challenge
    # {
    #     "type":  "reset_IWO",
    #     "x": 2.48,
    #     "y": 3.48,
    #     "z": 0.0,
    #     "roll": 0,
    #     "pitch": 0,
    #     "yaw": 180  # Starting facing South
    # },

    # {
    # "type": "drive_to_xyyaw_IWO",
    # "target_yaw": 270.0, 
    # "just_yaw": True
    # },

    # {
    #     "type": "linefollow_until_x_intersections",
    #     "side": "left",
    #     "timeout": 30,
    #     "speed": 0.4,
    #     "intersection_count": 1,
    # },

    # {
    # "type": "drive_to_xyyaw_IWO",
    # "target_yaw": 180.0, 
    # "just_yaw": True
    # },

    # # {
    # #     "type": "drive_straight",
    # #     "distance": 0.1, 
    # #     "speed": -0.1
    # # },

    # {
    #     "type": "servo_mid"
    # },

    # {
    #     "type": "servo_up"
    # },

    # # {
    # #     "type": "drive_to_xyyaw_IWO_fast",
    # #     "target_x": 2.48,
    # #     "target_y": 3.30,
    # #     "target_yaw": 200,   # Face South
    # #     "max_speed": 0.8,
    # #     "timeout": 15
    # # },

    # {
    #     "type": "drive_to_xyyaw_IWO_fast",
    #     "target_x": 2.48,
    #     "target_y": 3.30,
    #     "target_yaw": 180,   # Face South
    #     "max_speed": 0.8,
    #     "timeout": 15
    # },

    # # 3. druve to the Extra Time 2

    # {
    #     "type": "drive_to_xyyaw_IWO",
    #     "target_x": 0.50,
    #     "target_y": 3.4,
    #     "target_yaw": -40, # Face East
    #     "max_speed": 0.4,
    #     "timeout": 20
    # },


    # # {
    # #     "type": "drive_to_xyyaw_IWO_fast",
    # #     "target_x": 2.48,
    # #     "target_y": 3.30,
    # #     "target_yaw": 190,   # Face where balls are?
    # #     "max_speed": 0.4,
    # #     "timeout": 15
    # # },

    # Detect blue ball location
    {
        "type": "run_get_ball_sorting",
        "color": "blue"
    },

    # Drive to Quadrant C approach point
    {
        "type": "drive_to_xyyaw_IWO_fast",
        "target_x": 1.13,
        "target_y": 2.90,
        "target_yaw": 270,   # Face West/South towards C
        "max_speed": 0.4,
        "timeout": 15
    },

    # Push into Quadrant C
    {
        "type": "drive_to_xyyaw_IWO_fast",
        "target_x": 1.13,
        "target_y": 2.66,
        "target_yaw": 270,   
        "max_speed": 0.4,
        "timeout": 15
    },

    # Release Blue Ball
    {
        "type": "servo_up"
    },

    # Red ball
    # Back out and turn around to face the scatter zone again
    {
        "type": "drive_to_xyyaw_IWO_fast",
        "target_x": 1.50, # Back up safely before turning
        "target_y": 3.00,
        "target_yaw": 190, # Look back at the balls
        "max_speed": 0.4,
        "timeout": 15
    },

    # Detect, approach, and capture the red ball
    {
        "type": "run_get_ball_sorting",
        "color": "red"
    },

    # Drive to Quadrant B approach point
    {
        "type": "drive_to_xyyaw_IWO_fast",
        "target_x": 1.97,
        "target_y": 2.27,
        "target_yaw": 180,   # Face South towards B
        "max_speed": 0.4,
        "timeout": 15
    },

    # Push into Quadrant B
    {
        "type": "drive_to_xyyaw_IWO_fast",
        "target_x": 1.97,
        "target_y": 2.22,
        "target_yaw": 180,   
        "max_speed": 0.4,
        "timeout": 15
    },

    # Release Red Ball
    {
        "type": "servo_up"
    },
    
    
    {
        "type": "drive_straight",
        "distance": -0.2, 
        "speed": -0.3
    }
]

#     # Drive to and capture blue ball

#     # Drive (1.13,2.90) and then face 270
#     {
#         "type": "drive_to_xyyaw_IWO_fast",
#         "target_x": 1.13,
#         "target_y": 2.90,
#         "target_yaw": 270,   # Face South
#         "max_speed": 0.4,
#         "timeout": 15
#     },

#     # Drive to (1.13, 2.66) no change in yaw
#     {
#         "type": "drive_to_xyyaw_IWO_fast",
#         "target_x": 1.13,
#         "target_y": 2.66,
#         "target_yaw": 270,   # Face South
#         "max_speed": 0.4,
#         "timeout": 15
#     },

#     # Servo up
#     {
#         "type": "servo_up"
#     },

#     # Turn to face where red ball was detected

#     # Drive to and capture red ball

#     # Drive to (1.97,2.27) and then face 180
#     {
#         "type": "drive_to_xyyaw_IWO_fast",
#         "target_x": 1.97,
#         "target_y": 2.27,
#         "target_yaw": 180,   # Face South
#         "max_speed": 0.4,
#         "timeout": 15
#     },

#     # Drive to (1.97,2.22) and no change in yaw
#     {
#         "type": "drive_to_xyyaw_IWO_fast",
#         "target_x": 1.97,
#         "target_y": 2.22,
#         "target_yaw": 180,   # Face South
#         "max_speed": 0.4,
#         "timeout": 15
#     },

#     # Servo up
#     {
#         "type": "servo_up"
#     },

# ]