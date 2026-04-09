"""
missions/mission_gates.py
---------------------
Route: Start -> line follow -> turn left -> gate 3 circle -> line follow (right)
    -> turn 90 left -> line follow -> turn right -> line follow -> Goal
"""

TOTAL_TIME       = 120
GOAL_TIME_BUFFER = 20

TASKS = [

    #Start, follow line up toward gate 3 area
    {
        "type":     "line_follow",
        "side":     "left",
        "speed":    0.2,
        "timeout":  30
    },

    # AREA 3
    # Drive straight
    {
        "type":     "drive_straight",
        "speed":    0.3,
        "distance": 0.35,
        "timeout":  15,
    },

    # Turn to heading 
    {
        "type":      "turn_to_heading",
        "heading_deg": -8, # 0 is the heading at the beginning ot the mission, so this is a relative turn
        "speed":     0.3,
        "tolerance_deg": 2,
        "timeout":   10,
    },

    # Drive straight
    {
        "type":     "drive_straight",
        "speed":    0.15,
        "distance": 0.2,
        "timeout":  15,
    },

    # Turn in circle
    {
    "type":      "drive_circle",
    "radius":    0.33,       # metres
    "speed":     0.3,      # m/s — keep low for small radii
    "direction": "left",    # 'left' or 'right'
    "timeout":   20,
    },

    # Turn to heading 
    {
        "type":      "turn_to_heading",
        "heading_deg": 10, # 0 is the heading at the beginning ot the mission, so this is a relative turn
        "speed":     0.3,
        "tolerance_deg": 2,
        "timeout":   10,
    },

    # Drive straight
    {
        "type":     "drive_straight",
        "speed":    0.15,
        "distance": 0.4,
        "timeout":  15,
    },
    # END AREA 3

    {
        "type":             "line_follow_brake_crossing",
        "side":             "right",
        "speed":            0.3,
        "brake_speed":      -0.15,
        "max_speed":        0.5,
        "timeout":          200,
        "recovery":         True,
        "recovery_time":    0.1,
        "stop_at_crossing": True,
        "crossing_count":   3
    },

    # Turn to heading 
    {
        "type":      "turn_to_heading",
        "heading_deg": 180,
        "speed": 0.3,
        "timeout":   10,
    },

    # Drive straight
    {
        "type":     "drive_straight",
        "speed":    0.3,
        "distance": 2,
        "timeout":  60,
    },

    # Turn to heading 
    {
        "type":      "turn",
        "angle_deg": -90,
        "speed": 0.2,
        "timeout":   10,
    },

    # Turn to heading 
    {
        "type":      "turn",
        "angle_deg": -90,
        "speed": 0.2,
        "timeout":   10,
    },

    # Drive straight
    {
        "type":     "drive_straight",
        "speed":    0.3,
        "distance": 3.5,
        "timeout":  15,
    },
    
    #####################

    {
        "type":             "line_follow_brake_crossing",
        "side":             "right",
        "speed":            0.4,
        "brake_speed":      -0.2,
        "max_speed":        0.4,
        "timeout":          200,
        "recovery":         True,
        "recovery_time":    1.0,
        "stop_at_crossing": True,
        "crossing_count":   4, 
    },

    # AREA 3
    # Turn to heading 
    {
        "type":      "turn_to_heading",
        "heading_deg": -110, # 0 is the heading at the beginning ot the mission, so this is a relative turn
        "speed":     0.4,
        "tolerance_deg": 2,
        "timeout":   10,
    },

    # Drive straight
    {
        "type":     "drive_straight",
        "speed":    3.5,
        "distance": 0.4,
        "timeout":  15,
    },

    # Follow line to Goal
    {
        "type":     "line_follow",
        "side":     "left",
        "speed":    0.4,
        "timeout":  30,
    }

    # Turn right before final approach DETECT TURN ???
]

