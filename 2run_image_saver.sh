clear
source install/local_setup.bash
ros2 service call /GDH_start_detect gd_ifc_pkg/srv/GDHStartDetectObject "{object_types: 10}"
ros2 run gdh_package image_saver_node
