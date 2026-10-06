source /opt/ros/jazzy/setup.bash
# src/gd_ifc_pkg_org 도 패키지 이름이 gd_ifc_pkg 라서 src 전체를 탐색하면 중복 에러가 남 -> 빌드할 경로만 지정
colcon build --paths src/gd_ifc_pkg src/gdh_package src/gdh_vlm_client src/gdh_speech_audio src/ros2_gopro --symlink-install --parallel-workers $(nproc)
