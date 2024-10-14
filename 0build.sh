source /opt/ros/humble/setup.bash
colcon build --packages-select gd_ifc_pkg  
colcon build --packages-select gdh_package
colcon build --packages-select gdh_speech_audio