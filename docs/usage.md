# 사용 설명서 (ur_pick_place)

UR 로봇(ROS 2 Humble + Gazebo Classic)에 **평행 그리퍼**와 **손목 RGB-D 카메라**를 장착한 시뮬레이션 패키지입니다.
UR 공식 패키지(`ur_description`, `ur_simulation_gazebo`)를 수정하지 않고, 이 패키지의 URDF/컨트롤러 설정만 덮어씌워 사용합니다.

## 구성
| 파일 | 내용 |
|---|---|
| `urdf/ur_gripper.urdf.xacro` | UR 본체(`ur.urdf.xacro`) + 그리퍼 + 카메라 조립 |
| `urdf/parallel_gripper.xacro` | 2지 평행 그리퍼 (`tool0`에 장착, 손가락 2개 prismatic) |
| `urdf/wrist_camera.xacro` | 손목 깊이 카메라 (640x480, FOV 69°, 0.05–3 m) |
| `config/ur_gripper_controllers.yaml` | 팔 `joint_trajectory_controller` + `gripper_controller` |
| `launch/ur_gripper_sim.launch.py` | Gazebo + RViz 실행 |
| `scripts/gripper.py` | 그리퍼 열기/닫기 명령 |
| `scripts/pick_place.py` | 큐브 픽앤플레이스 자동 실행 |
| `scripts/ur_kinematics.py` | UR5e 순/역기구학 (numpy) |
| `src/grasp_attacher.cpp` | Gazebo 잡기 안정화 플러그인 (`/grasp/attach`, `/grasp/detach`) |
| `models/table.sdf`, `models/cube.sdf` | 테이블(윗면 z=0.20 m), 4 cm 큐브 |

## 설치
```bash
sudo apt install ros-humble-ur ros-humble-ur-simulation-gazebo \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-ros2-controllers
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src && git clone https://github.com/hyungyu0820/ur5e-conveyor-sorting.git ur_pick_place
cd ~/ros2_ws && colcon build --packages-select ur_pick_place && source install/setup.bash
```

## 실행
```bash
bash ~/ros2_ws/src/ur_pick_place/run_sim.sh                    # 빌드 + 정리 + 실행, 로그: ~/ros2_ws/log/sim/latest.log
bash ~/ros2_ws/src/ur_pick_place/run_sim.sh ur5e gazebo_gui:=false launch_rviz:=false   # 화면 없이
ros2 launch ur_pick_place ur_gripper_sim.launch.py spawn_objects:=false                 # 테이블/큐브 없이
```

컨트롤러 확인 (3개 모두 `active` 여야 함):
```bash
ros2 control list_controllers
```

픽앤플레이스 (다른 터미널에서, `source ~/ros2_ws/install/setup.bash` 후):
```bash
ros2 run ur_pick_place pick_place.py                                   # (0.45,-0.25) -> (0.45,0.25)
ros2 run ur_pick_place pick_place.py --pick 0.45 0.25 --place 0.6 0.0
gz model -m cube -p                                                    # 큐브 실제 위치 확인
```
물건 생성 / 삭제, 이름으로 집기:
```bash
ros2 run ur_pick_place spawn_object.py box_blue 0.40 -0.10 --color Blue   # 4 cm 큐브 생성
ros2 run ur_pick_place pick_place.py --object box_blue --place 0.35 0.25  # 위치는 Gazebo에서 자동으로 읽음
ros2 run ur_pick_place spawn_object.py box_blue --delete                  # 삭제
```
물건끼리는 6 cm 이상 띄워 두세요 (그리퍼가 옆 물건을 잡을 수 있음).

`pick_place.py`의 좌표는 `ur5e` 기준입니다 (`ur_kinematics.py`의 DH 파라미터).

그리퍼 (손가락 이동량 0.0 = 열림(간격 90 mm), 0.04 = 닫힘(간격 10 mm)):
```bash
ros2 run ur_pick_place gripper.py close
ros2 run ur_pick_place gripper.py open
ros2 run ur_pick_place gripper.py 0.02
```

카메라 토픽:
```bash
ros2 topic list | grep wrist_camera
#  /wrist_camera/image_raw        RGB 영상
#  /wrist_camera/depth/image_raw  깊이 영상
#  /wrist_camera/points           포인트클라우드
#  /wrist_camera/camera_info
ros2 run rqt_image_view rqt_image_view /wrist_camera/image_raw
```

## 주요 좌표계
- `tool0` → `gripper_base_link` → `finger_left/right_link`
- `gripper_tcp` : 손가락 끝 사이 중심 (tool0 기준 z +8 cm) — MoveIt 엔드이펙터로 사용 예정
- `camera_optical_frame` : 카메라 광학 좌표계 (z = 보는 방향 = 그리퍼 방향)

## 컨베이어 분류 셀 (물류 시연)
```bash
bash ~/ros2_ws/src/ur_pick_place/run_sim.sh ur5e scene:=conveyor     # 터미널 1
ros2 run ur_pick_place conveyor_sorter.py                            # 터미널 2, Ctrl+C로 정지
ros2 run ur_pick_place conveyor_sorter.py --cycles 10                # 10개만
```
- 컨베이어(`src/conveyor_belt.cpp`): 벨트 위 물건을 0.1 m/s로 운반, 광센서(노란 기둥, world y=0)에 물건이 오면 자동 정지
  - `ros2 service call /conveyor/power std_srvs/srv/SetBool "{data: true}"` 수동 기동
  - `/conveyor/running`, `/conveyor/object_at_sensor` (std_msgs/Bool)
- 피더(`scripts/conveyor_scene.py`): 빨강/초록/파랑 분류함 생성 후, 벨트 25 cm 이동마다 랜덤 색 큐브 공급(무한). 30개가 넘으면 오래된 것부터 삭제
- 분류기(`scripts/conveyor_sorter.py`): 센서 위에서 대기 → 손목 카메라로 색·위치 인식 → 집기 → 벨트 재가동 → 색별 분류함(3x3 칸에 차례로)
- 인식(`scripts/color_detector.py`): `detect(rgb) -> [Detection(label, u, v, area)]`. 다른 인식기(예: 비전-언어 모델)로 바꿀 때 이 함수만 교체
- 배치(`scripts/conveyor_layout.py`): 벨트·센서·분류함 좌표

## 문제 해결
| 증상 | 원인 / 해결 |
|---|---|
| `list_controllers` 응답 없음, spawner가 `Failed getting a result` | gzserver가 죽음(로그에 `process has died ... exit code -11`). WSL2 GPU 드라이버에서 깊이 카메라가 segfault → `run_sim.sh`가 `LIBGL_ALWAYS_SOFTWARE=1`로 해결. GPU를 쓰려면 `GZ_SW_RENDER=0 bash run_sim.sh` |
| gzserver가 바로 `exit code 255` | 이전 gzserver가 남아 있음 → `pkill -9 -x gzserver` (`run_sim.sh`가 자동 수행) |
| `ros2 topic list`에서 `!rclpy.ok()` | ros2 데몬 꼬임 → `ros2 daemon stop; ros2 daemon start` |
| 월드에 조명/바닥 없음 (`Unable to find uri[model://sun]`) | `source /usr/share/gazebo/setup.sh` 필요 (`run_sim.sh`가 수행) |
| 로그의 `Parameter 'hold_joints' has already been declared` | 팔/그리퍼가 각각 `<ros2_control>` 블록을 가져서 생기는 메시지. 동작에는 영향 없음 |
