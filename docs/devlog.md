# 개발 일지

UR5e + 평행 그리퍼 + 손목 카메라 모델이 Gazebo에 보이기만 하고 **명령은 하나도 듣지 않는 상태**에서 시작해서, 컨베이어 비전 분류 셀을 완성하기까지의 과정입니다.
각 단계는 **증상 → 원인 분석 → 해결 → 검증** 순서로 기록했습니다.

## 진행 방식

- **문제를 증거로 좁히기**: 추측으로 코드를 고치지 않고, 먼저 로그를 저장해서 실패 지점을 찾고, 구성 요소를 하나씩 꺼보며(이분 탐색) 원인을 특정
- **눈으로 보지 말고 숫자로 검증**: "움직이는 것 같다"가 아니라 액션 결과 코드, TF 값, Gazebo 실제 좌표로 확인
- **작게 만들고 바로 시험**: 단일 픽앤플레이스 → 물건 생성 → 컨베이어 순서로 기능을 쌓고, 단계마다 시뮬레이션에서 실행
- **공식 패키지는 수정하지 않음**: UR 공식 launch / description을 그대로 재사용하고, 이 패키지의 설정만 덮어씀
- AI 코딩 도구(Claude Code)와 페어 프로그래밍으로 진행

---

## 1. controller_manager가 응답하지 않음

**증상**
- `ros2 control list_controllers` → `/controller_manager/list_controllers` 서비스 무응답
- `ros2 topic list` → `xmlrpc Fault "!rclpy.ok()"`

**분석**

시뮬레이션을 화면 없이 실행하고 출력을 파일로 저장해서 보니, 컨트롤러 문제가 아니었습니다.

```
[gazebo_ros2_control]: Loaded gazebo_ros2_control.
[controller_manager]: Loading controller 'joint_state_broadcaster'
[ERROR] [gzserver-4]: process has died [pid 2461, exit code -11, ...]
```

- exit code -11 = **segfault**. ROS 2 + Gazebo에서는 controller_manager가 gzserver 프로세스 안에서 돌기 때문에, gzserver가 죽으면 서비스도 사라집니다.
- 의심 요소를 하나씩 꺼서 확인:

| 시험 | 결과 |
|---|---|
| 원래 상태 | gzserver segfault |
| 손목 깊이 카메라 제거 | 컨트롤러 3개 모두 `active` |
| 카메라 유지 + `LIBGL_ALWAYS_SOFTWARE=1` | 컨트롤러 3개 모두 `active`, 카메라 토픽 정상 |

→ **WSL2의 GPU(D3D12) OpenGL 드라이버와 Gazebo 깊이 카메라 센서의 충돌**로 특정했습니다.

**추가로 발견한 원인**
- 이전 실행의 gzserver가 남아 있으면 새 gzserver가 포트 11345를 못 잡고 바로 `exit code 255`로 종료
- ros2 CLI 데몬이 꼬이면 `!rclpy.ok()` 에러 발생

**해결**: `run_sim.sh`에서 다음을 자동으로 처리
1. 이미 실행 중인 시뮬레이션이 있으면 건드리지 않고 중단
2. 남은 gzserver / gzclient 정리
3. 소프트웨어 렌더링 (`GZ_SW_RENDER=0`이면 GPU 사용)
4. ros2 데몬 재시작
5. 로그를 `~/ros2_ws/log/sim/latest.log`에 저장

## 2. 팔 · 그리퍼 · 카메라 동작 확인

| 항목 | 확인 방법 | 결과 |
|---|---|---|
| 컨트롤러 | `ros2 control list_controllers` | `joint_trajectory_controller`, `gripper_controller`, `joint_state_broadcaster` 모두 active |
| 팔 | `FollowJointTrajectory` 액션 | `SUCCEEDED`, error_code 0 |
| 그리퍼 | `/joint_states`의 손가락 값 | close 0.040, open 0.000 |
| 카메라 | `ros2 topic hz` | RGB, 깊이, 포인트클라우드 모두 발행 |

## 3. 픽앤플레이스

### 운동학: MoveIt 대신 직접 구현
그리퍼가 붙은 모델에 맞는 MoveIt 설정이 없어서, 동작 원리를 이해할 겸 직접 구현했습니다.
- **FK**: UR5e DH 파라미터. URDF의 `base_link`가 DH 기준 좌표계에 대해 z축으로 180° 회전돼 있어서 이를 반영
- **검증**: 관절 `[0, -1.57, 1.57, -1.57, -1.57, 0]`에서 FK 결과와 `tf2_echo world tool0` 값이 **위치 (0.492, 0.133, 0.488)와 자세 모두 일치**
- **IK**: 수치 야코비안 + 감쇠 최소자승(DLS). 현재 자세를 시작값으로 써서 가까운 해를 선택
- 직선 경로: 내려가기·올라가기는 10개 지점으로 나눠서 각각 IK를 풀고, 하나의 궤적으로 전송

### 잡기 안정화: grasp 플러그인
- **문제**: `gazebo_ros2_control`의 position 명령은 관절을 매 스텝 **순간이동(SetPosition)** 시킵니다. 그래서 손가락이 물체를 파고들거나 마찰만으로는 미끄러집니다.
- 기존 해결책인 `gazebo_grasp_plugin`은 ROS 1 전용이었습니다.
- **해결**: Gazebo 모델 플러그인 `grasp_attacher.cpp`를 직접 작성
  - `/grasp/attach`: 두 손가락 중간점에서 5 cm 안의 가장 가까운 물체를 손목 링크에 fixed joint로 고정
  - `/grasp/detach`: 해제
  - 물리 스레드와의 경쟁을 막기 위해 physics update mutex를 잡고 joint를 생성·해제
  - URDF의 fixed joint는 Gazebo에서 하나로 합쳐지므로, `gripper_base_link`가 아니라 `wrist_3_link`를 손목 링크로 사용

**검증**: (0.45, -0.25) → (0.45, 0.25)로 옮긴 뒤 Gazebo 실제 좌표 (0.450, 0.250, 0.220), 두 번째 (0.600, 0.000, 0.220)

## 4. 물건 생성 + 이름으로 집기

- `spawn_object.py`: `/spawn_entity` 서비스로 색을 지정해서 큐브 생성, `--delete`로 삭제
- `pick_place.py --object NAME`: Gazebo에서 물체 좌표를 읽어 집고, 쌓인 물체처럼 높이가 달라도 그 높이에서 집음
- 파란·초록 큐브를 생성해서 빨간 큐브 양옆으로 한 줄 정렬 → 목표 대비 1 mm 이내

## 5. 컨베이어 비전 분류 셀

### 설계
| 구성 요소 | 구현 |
|---|---|
| 컨베이어 | `conveyor_belt.cpp`: 벨트 영역 안 물체의 속도를 매 스텝 벨트 속도로 설정. 광센서 구간에 물체가 들어오면 자동 정지 |
| 재가동 | `/conveyor/power` (SetBool). 재가동할 때 센서를 다시 대기 상태로 돌려서, 센서에 물체가 남아 있으면 즉시 다시 정지 |
| 피더 | `conveyor_scene.py`: 벨트 이동 거리를 적분해서 25 cm마다 공급. 이름에 정답 색을 넣어 사후 채점에 사용 (분류기는 이름을 쓰지 않음) |
| 인식 | `color_detector.py`: 채널 우세 임계값 + 연결 요소(connected components). 이웃 물체가 하나로 합쳐지지 않도록 덩어리를 분리 |
| 3D 위치 | 픽셀 → 깊이(5×5 중앙값) → 카메라 내부 파라미터로 역투영 → TF로 `world` 변환. 센서에서 8 cm 넘게 떨어진 검출(대기 중인 다음 물체)은 무시 |
| 분류기 | `conveyor_sorter.py`: 집고 올린 직후 벨트를 재가동해서, 로봇이 분류하는 동안 다음 물체가 이동하도록 함 |

### 테스트 중 발견하고 고친 버그

| # | 증상 | 원인 | 해결 |
|---|---|---|---|
| 1 | 월드에 조명·바닥이 없음 (`Unable to find uri[model://sun]`) | `gazebo_ros` launch가 `GAZEBO_MODEL_PATH`를 덮어써서 기본 모델 경로가 빠짐 | `run_sim.sh`에서 `/usr/share/gazebo/setup.sh` 실행 |
| 2 | 첫 물체가 바닥으로 떨어짐 | 피더가 컨베이어 모델보다 먼저 물체를 생성 (경쟁 조건) | 컨베이어 플러그인의 첫 상태 메시지를 받은 뒤에만 공급 |
| 3 | 같은 색 물체가 분류함 중앙에 탑처럼 쌓임 (z 0.18 → 0.26) | 매번 같은 지점에 놓음. 계속 쌓이면 그리퍼와 충돌 | 분류함 안 3×3 칸을 차례로 사용 |
| 4 | (코드 작성 중 수치 검증으로 발견) 뒤쪽 분류함으로 갈 때 손목이 불필요하게 회전할 위험 | IK 시작값에서 wrist_3 보정의 부호가 반대 (공구 z축이 아래를 향하므로 부호가 바뀜) | 부호 수정. 모든 분류함 칸 × 2개 높이에서 IK 수렴 확인 |

### 검증
- 12개 연속 분류 → 이름의 정답 색과 실제 분류함 위치를 Gazebo 좌표로 자동 채점: **정답 12 / 오답 0**
- 인식 위치 (0.499, -0.019) / 실제 (0.500, -0.020): **약 1 mm**
- 12개 228초 → **물건당 약 19초**

## 배운 점

- ROS 2 + Gazebo에서 "서비스 무응답"은 대부분 **서비스를 제공하는 프로세스가 죽은 것**입니다. 먼저 로그에서 `process has died`를 찾는 것이 빠릅니다.
- 시뮬레이터의 제어 방식(position = 순간이동)이 잡기 물리에 직접 영향을 줍니다. 시뮬레이션 결과를 실기에 옮길 때 차이가 나는 지점입니다.
- 인식 모듈을 `detect(rgb)` 함수 하나로 분리해 두면, 규칙 기반 인식을 VLM으로 바꾸는 실험이 쉬워집니다.
- 수동으로 보는 것보다 자동 채점 스크립트로 확인할 때 숨은 문제(분류함 적층)가 드러났습니다.
