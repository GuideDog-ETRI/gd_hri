source /opt/ros/humble/setup.bash
colcon build --packages-select gd_ifc_pkg gdh_package gdh_speech_audio ros2_gopro --symlink-install --parallel-workers $(nproc)
