std_msgs/Header header

# Sensor Type # 1: 3D, 2 ~: 2D-image
uint8 sensor_id 

# Ego GPS at Sensor Input
# sensor_msgs/NavSatFix gps
# sensor_msgs/Imu imu

# Ego Odometry from World (Local World)
nav_msgs/Odometry odometry

# Object 
Detection[] detections