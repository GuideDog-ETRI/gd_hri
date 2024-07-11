
# Past Trajectory

# Single-modal or Multi-modal 
uint8 id

# Future Trajectory
geometry_msgs/Point[] poses

# Track Information # vision_msgs/ObjectHypothesis

# Object Type
uint8 type
uint8 PEDESTRIAN=0
uint8 CAR=1
uint8 CYCLIST=2

# Instance Point Cloud # points for 3D # pixels for 2D
# sensor_msgs/PointCloud2 points


