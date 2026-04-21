from missions.tasks import TaskSetServo, TaskSpinUntilCameraSees, TaskApproachCube, TaskDrive

# Mission logic for catching the cube ID=53
def get_cube_catch_mission():
    return [
        TaskSetServo(servo_id=1, position=-930, wait_time=0.5), # Ensure arm is UP
        TaskSpinUntilCameraSees(marker_type="cube_center"),     # Spin until we see the cube
        TaskApproachCube(stop_distance=0.29),                   # 29cm is arm physical reach
        TaskSetServo(servo_id=1, position=150, wait_time=1.0),  # Bring arm DOWN to catch it
        # Jiggle
        TaskDrive(forward_vel=0.0, turn_vel=0.4, duration=1.0),
        TaskDrive(forward_vel=0.0, turn_vel=-0.4, duration=2.0),
        # Release servo tension
        TaskSetServo(servo_id=1, position=10000, wait_time=0.1) 
    ]
