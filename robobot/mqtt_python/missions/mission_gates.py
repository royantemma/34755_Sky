"""
missions/mission_gates.py
---------------------
Route: Start -> line follow -> turn left -> gate 3 circle -> line follow (right)
    -> turn 90 left -> line follow -> turn right -> line follow -> Goal
"""

TOTAL_TIME       = 120
GOAL_TIME_BUFFER = 20

TASKS = [

    # Start, follow line up toward gate 3 area
    {
        "type":     "line_follow",
        "side":     "left",
        "speed":    0.2,
        "timeout":  30
    },

    # Turn left before gate 3
    {
        "type":      "turn",
        "angle_deg": 90, # ??? angle
        "speed":     0.3,
        "timeout":   10,
    },

    # Gate 3 circle (odometry arc) (implement run_drive_arc later)
    {
        "type":     "drive_arc",  # NOT YET IMPLEMENTED
        "speed":    0.15,
        "timeout":  15,
    },

    # Follow line
    {
        "type":     "line_follow",
        "side":     "right",
        "speed":    0.2,
        "timeout":  30
    },

    # Turn 90 degrees left
    {
        "type":      "turn",
        "angle_deg": 90,
        "speed":     0.3,
        "timeout":   10,
    },

    # Follow line
    {
        "type":     "line_follow",
        "side":     "left",
        "speed":    0.2,
        "timeout":  20
    },

    # Turn right before final approach DETECT TURN ???
    {
        "type":      "turn",
        "angle_deg": -90, # angle ???
        "speed":     0.3,
        "timeout":   10,
    },

    # Follow line to Goal
    {
        "type":     "line_follow",
        "side":     "left",
        "speed":    0.2,
        "timeout":  30,
    },
]