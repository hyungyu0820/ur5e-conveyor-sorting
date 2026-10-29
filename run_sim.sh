#!/bin/bash
# 빌드 + 환경설정 + Gazebo 실행을 한 번에 수행
# 사용법: bash ~/ros2_ws/src/ur_pick_place/run_sim.sh [ur_type] [추가 launch 인자...]
#   예) bash run_sim.sh ur5e scene:=conveyor          # 컨베이어 분류 셀
#   예) bash run_sim.sh ur5e gazebo_gui:=false launch_rviz:=false
# 로그: ~/ros2_ws/log/sim/latest.log
set -e
UR_TYPE=${1:-ur5e}
shift || true

# 0) 다른 터미널에서 이미 시뮬레이션이 돌고 있으면 건드리지 않고 종료
if pgrep -f "ros2 launch ur_pick_place" >/dev/null; then
  echo "이미 시뮬레이션이 실행 중입니다. 그 터미널에서 Ctrl+C로 끈 뒤 다시 실행하세요." >&2
  exit 1
fi

# 1) 이전 실행에서 남은 Gazebo 프로세스 정리 (남아 있으면 새 gzserver가 바로 죽음)
pkill -9 -x gzserver 2>/dev/null || true
pkill -9 -x gzclient 2>/dev/null || true
sleep 1

# 2) WSL2: GPU(D3D12) OpenGL 드라이버에서 깊이 카메라가 gzserver를 segfault 시키므로
#    소프트웨어 렌더링 사용. GPU로 돌리고 싶으면 GZ_SW_RENDER=0 bash run_sim.sh
if [ "${GZ_SW_RENDER:-1}" = "1" ]; then
  export LIBGL_ALWAYS_SOFTWARE=1
fi

# Gazebo 기본 모델 경로(sun, ground_plane). 없으면 월드에 조명과 바닥이 빠짐
source /usr/share/gazebo/setup.sh
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select ur_pick_place
source ~/ros2_ws/install/setup.bash

# 3) ros2 CLI 데몬이 꼬여 있으면 재시작 (ros2 topic list 의 "!rclpy.ok()" 에러 방지)
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

mkdir -p ~/ros2_ws/log/sim
ros2 launch ur_pick_place ur_gripper_sim.launch.py ur_type:=$UR_TYPE "$@" 2>&1 | tee ~/ros2_ws/log/sim/latest.log
