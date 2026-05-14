"""
missions/mission_gates.py
---------------------
Test Route: Start -> drive 3.5m -> turn left 90 -> line follow to (5.41, -2.00) 
            -> drive to extra time pt 1 -> return to xy -> line follow ramp
"""

TOTAL_TIME       = 120
GOAL_TIME_BUFFER = 20

TASKS = [

    # Pre luggage challenge. Ball holder. Arrive from ramp
    

    # 1. Reset Odometry at the starting point of the Luggage Challenge
    {
        "type":  "reset_IWO",
        "x": 2.48,
        "y": 2.90,
        "z": 0.0,
        "roll": 0,
        "pitch": 0,
        "yaw": 180  # Starting facing South
    },

    # 2. clear the ball holder obstacle
    {
        "type": "drive_to_xyyaw_IWO_fast",
        "target_x": 2.48,
        "target_y": 3.30,
        "target_yaw": 180,   # Face South
        "max_speed": 0.8,
        "timeout": 15
    },

    # 3. druve to the Extra Time 2

    {
        "type": "drive_to_xyyaw_IWO",
        "target_x": 0.50,
        "target_y": 3.4,
        "target_yaw": -90, # Face East
        "max_speed": 0.4,
        "timeout": 20
    },

    # 4. Navigate down to the Conveyor intercept Point
    {
        "type": "drive_to_xyyaw_IWO_fast",
        "target_x": 0.15,
        "target_y": 2.33,
        "target_yaw": 180, # Perpendicular to conveyor (Facing South)
        "max_speed": 0.6,
        "timeout": 20
    },

    # 5. catch box 20
    # what should the timeout be?
    {
        "type": "luggage_intercept",
        "target_box_id": 20,
        "strike_zone_x": 30.0,
        "timeout": 90
    },

    # 6. Transport box 20 to Quadrant A
    {
        "type": "drive_to_xyyaw_IWO_fast",
        "target_x": 0.50,  
        "target_y": 1.50,
        "target_yaw": -90,
        "max_speed": 0.4,
        "timeout": 20
    },
    {
    "type": "drive_until_line_detection",
    "speed": 0.2,
    "timeout": 30
}   ,
    {
    "type": "drive_to_xyyaw_IWO",
    "target_yaw": 0.0, 
    "just_yaw": True
    },
    {
    "type": "linefollow_until_x_intersections",
    "side": "left",
    "timeout": 30,
    "speed": 0.4,
    },
    {
    "type": "drive_to_xyyaw_IWO",
    "target_yaw": 90, 
    "just_yaw": True
    },


    {
        "type": "drive_to_xyyaw_IWO_fast",
        "target_x": 0.90,  
        "target_y": 1.90,
        "target_yaw": 30, # need to correct the yaw?
        "max_speed": 0.4,
        "timeout": 20
    },
    {
        "type": "drive_to_xyyaw_IWO_fast",
        "target_x": 0.90,  
        "target_y": 1.90,
        "target_yaw": 30, # need to correct the yaw?
        "max_speed": 0.4,
        "timeout": 20
    },

    # 7. drop off box 20 in quadrant A
    {
        "type": "servo_up"
    },
]