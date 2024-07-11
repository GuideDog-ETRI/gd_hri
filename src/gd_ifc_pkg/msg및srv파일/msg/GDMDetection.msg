# Detection Information # vision_msgs/ObjectHypothesis
int64 id # identification number
float64 score # detection confidence score

# Object State # x, y, z <= geomtery_msgs/Pose.msg
geometry_msgs/Point center # ignore z for image
geometry_msgs/Quaternion orientation # ignore for image 
# float64 orientation

# Appearance feature # geometry_msgs/Vector3
float64 height
float64 width
float64 length # ignore for image 

# Object Type
uint8 type
uint8 PEDESTRIAN=0
uint8 CAR=1
uint8 CYCLIST=2

# Feature Vector
float64[] appearance_feature

# Instance Point Cloud # points for 3D # pixels for 2D
sensor_msgs/PointCloud2 points
