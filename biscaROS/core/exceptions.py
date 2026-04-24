# biscaROS/core/exceptions.py

class MissionAborted(Exception):
    """Raised by task modules to instantly terminate a mission script."""
    pass