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

# [2026-10-23] 컨테이너 안에 오디오 소켓이 없으면 STT가 마이크(샥즈)를 전혀 못 씀
#   (재부팅 때 컨테이너가 사용자 오디오보다 먼저 시작된 경우. ~/run_gdh_all*.sh 는 자동으로 컨테이너 재시작)
if [ ! -S /run/user/2002/pulse/native ]; then
  echo "⚠️  /run/user/2002/pulse/native 없음: 음성 인식(STT)이 마이크를 쓸 수 없습니다."
  echo "    호스트에서 'docker restart thor_gdh' 후 다시 실행하세요."
fi

PULSE_SERVER=unix:/run/user/2002/pulse/native pactl set-default-source bluez_input.C0_86_B3_CB_A2_16.0 2>/dev/null || true

ros2 launch ./launch/gdh_test.launch.xml
