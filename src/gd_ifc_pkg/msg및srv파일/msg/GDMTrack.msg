# Track Information # vision_msgs/ObjectHypothesis
int64 id # identification number
float64 score # Track confidence score from status, count, modality
uint8[] sensor_ids # 1: 3D, 2: 2D-1, 3: 2D-2


# Object State # x, y, z <= geomtery_msgs/Pose.msg
geometry_msgs/Point center 
geometry_msgs/Quaternion orientation 
# float64 orientation

# Appearance feature # geometry_msgs/Vector3
float64 height
float64 width
float64 length  

# velocity with linear and angular velocity (rx, ry, rz)
geometry_msgs/Twist velocity

# Object Type
uint8 type
uint8 PEDESTRIAN=0
uint8 CAR=1
uint8 CYCLIST=2

# Instance Point Cloud # points for 3D # pixels for 2D
sensor_msgs/PointCloud2 points


