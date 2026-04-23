"""
missions/mission_gates.py
---------------------
Test Route: Start -> drive 3.5m -> turn left 90 -> line follow to (5.41, -2.00) 
            -> drive to extra time pt 1 -> return to xy -> line follow ramp
"""

TOTAL_TIME       = 120
GOAL_TIME_BUFFER = 20

TASKS = [

    # 1. Drive forward 3.5 meters
    {
        "type":     "drive_straight",
        "speed":    0.3,
        "distance": 3.5,
        "timeout":  40,
    },

    # 2. Turn left 90 degrees
    # Positive angles are counter-clockwise (Left)
    {
        "type":      "turn",
        "angle_deg": 70, 
        "speed":     0.4,
        "timeout":   15,
    },

    # 3. Approach line & Follow to (5.41, -2.33)
    {
        "type":     "line_follow_until_xy",
        "side":     "right",
        "speed":    0.3,
        "target_x": 5.41,
        "target_y": -2.33,
        "distance_tolerance": 0.1,
        "timeout":  40
    },

    # 4. Adjust to zero heading
    {
        "type":          "turn_to_heading",
        "heading_deg":   0, 
        "speed":         0.4,
        "tolerance_deg": 1.5,
        "timeout":       10
    },

    # 5. Reverse to Extra Time Point 1
    {
        "type":        "drive_straight_heading",
        "speed":       -0.3,  # Negative = Reverse
        "distance":    2.10,  # 5.41 - 3.31
        "heading_deg": 0,     # Maintain 0 degrees while backing up
        "timeout":     15
    },

    # 6. Drive Forward to return to the exit point
    {
        "type":        "drive_straight_heading", 
        "speed":       0.3,   # Positive = Forward
        "distance":    2.10, 
        "heading_deg": 0,     # Maintain 0 degrees while driving forward
        "timeout":     15
    },

    # 7. Catch the line and continue up the ramp
    {
        "type":     "line_follow",
        "side":     "right",
        "speed":    0.4,
        "timeout":  30
    },

]
