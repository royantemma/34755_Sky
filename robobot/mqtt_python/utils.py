from enum import Enum

class JunctionDirection(Enum):
    NONE = "none" # no junction detected
    SHARP_LEFT = "sharp_left" # sharp (~90°) left junction
    LEFT = "left" # left junciton
    RIGHT = "right"
    SHARP_RIGHT = "sharp_right"

class WindowDirection(Enum):
    LEFT = "left"
    TOP = "top"
    RIGHT = "right"