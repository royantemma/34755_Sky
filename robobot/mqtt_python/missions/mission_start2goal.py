"""
missions/mission_start2goal.py
---------------------
Route: Start -> follow line (-> gate 2) -> Goal

"""

TOTAL_TIME       = 120   # seconds allowed for the whole mission
GOAL_TIME_BUFFER = 20    # if less than this many seconds remain, skip to goal

TASKS = [

    # The line runs along the left edge of the track coming out of Start.
    {
        "type":     "line_follow",
        "side":     "right",      # follow the RIGHT edge of the line
        "speed":    0.2,
        "timeout":  3,
    }
]