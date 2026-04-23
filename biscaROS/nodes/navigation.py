"""
The navigation node is responsible for estimating the robot's position and orientation on the track, using the Kalman filter from Henrik.
It subscribes to the MQTT topics that give measurements : gyro, wheel odometry and vision (for now only ArUco).
It publishes its estimates to another MQTT topic.
"""