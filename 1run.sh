clear
source /home/gdr/.bashrc
source /home/gdr/gdh/workspace/gd_hri/install/setup.bash

# [2026-06-02] 추가: DISPLAY 소켓 자동 교정
# 원본: ros2 launch 직전에 DISPLAY 설정 없음
# 문제: run_gdh_all.sh가 gnome-session 환경에서 DISPLAY=:1을 읽어오지만
#       실제 X11 소켓은 /tmp/.X11-unix/X0 에만 존재해 cv2.imshow가 SIGABRT로 충돌
# 수정: 현재 DISPLAY 소켓이 없으면 X0 소켓으로 교정
if [ ! -S "/tmp/.X11-unix/X${DISPLAY#:}" ] && [ -S "/tmp/.X11-unix/X0" ]; then
  export DISPLAY=:0
fi

PULSE_SERVER=unix:/run/user/2002/pulse/native pactl set-default-source bluez_input.C0_86_B3_CB_A2_16.0 2>/dev/null || true

ros2 launch ./launch/gdh_test.launch.xml
