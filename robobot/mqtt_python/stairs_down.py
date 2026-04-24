# stairs_down.py
#
# Mission to make the robot go down the stairs.

def get_stairs_down_mission():
    """
    Returns the list of tasks for the 'stairs down' mission.
    """
    mission = [
        # TODO: Add tasks here
        {"type": "drive_straight", "distance": 0.1, "speed": 0.1},
    ]
    return mission

if __name__ == "__main__":
    # This part is for testing the mission logic
    # It will not be executed when run from the main mission runner
    from mission_runner import MissionRunner
    
    print("Testing stairs_down mission")
    # To run this test, you would need to initialize all the required
    # robot components (pose, service, etc.) here.
    # For now, this just prints the mission tasks.
    
    stairs_mission = get_stairs_down_mission()
    print("Mission tasks:")
    for task in stairs_mission:
        print(f"- {task}")

    # Example of how you might run it (requires full robot setup)
    # runner = MissionRunner(stairs_mission, total_time=300, goal_time_buffer=30)
    # runner.run()
