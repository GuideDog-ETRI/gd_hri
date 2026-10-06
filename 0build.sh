source /opt/ros/jazzy/setup.bash
colcon build --packages-select gdh_package gdh_vlm_client gdh_speech_audio ros2_gopro --symlink-install --parallel-workers $(nproc)
