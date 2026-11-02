#!/bin/bash
# 시뮬레이션 전체 종료 (run_sim.sh로 켠 Gazebo, launch, 피더, spawner 등)
# 사용법: bash ~/ros2_ws/src/ur_pick_place/stop_sim.sh
# 검색어를 이 파일 안에 두어서, 이 스크립트를 실행한 터미널 명령 자체가 잡히지 않게 함

launch_pids=$(pgrep -f "^/usr/bin/python3 [^ ]*/ros2 launch ur_pick_place")
[ -n "$launch_pids" ] && kill -INT $launch_pids
sleep 5

for pattern in "^bash .*run_sim.sh" "^/usr/bin/python3 [^ ]*/ros2 launch ur_pick_place" \
  "lib/ur_pick_place/conveyor_scene.py" "lib/robot_state_publisher/robot_state_publisher" \
  "lib/controller_manager/spawner" "gazebo_ros/spawn_entity.py"; do
  for p in $(pgrep -f "$pattern"); do
    [ "$p" != "$$" ] && kill -9 "$p" 2>/dev/null
  done
done
pkill -9 -x gzserver 2>/dev/null
pkill -9 -x gzclient 2>/dev/null
pkill -9 -x rviz2 2>/dev/null
sleep 1

left=$(pgrep -x gzserver; pgrep -f "^/usr/bin/python3 [^ ]*/ros2 launch ur_pick_place")
if [ -z "$left" ]; then echo "시뮬레이션 종료 완료"; else echo "남은 프로세스: $left"; fi
