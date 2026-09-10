# 실물 로봇 연결 검토 (항목 9)

**결론부터.** 로봇 팔은 URDF 인자 하나로 바꿔 끼울 수 있게 이미 되어 있다.
막히는 곳은 로봇이 아니라 **주변 장치와 좌표**다. 순서대로 적는다.

> 이 문서는 "무엇을 바꿔 끼워야 하는가"의 검토다. 그 뒤에 실제로 들어간
> 자리(런치 인자, I/O 토픽 계약, `twin_mirror`)와 무엇이 아직 비어 있는지는
> `docs/digital_twin.md`에 있다. 호스트 설치는 `docs/native_ubuntu24.md`.

---

## 1. 지금 구조에서 실물과 시뮬을 가르는 곳

`hardware` 인자 한 곳이다. `box_cell_robot.urdf.xacro`:

| 값 | 플러그인 | 쓰임 |
|---|---|---|
| `gazebo` (기본) | `gz_ros2_control/GazeboSimSystem` | 지금 돌리는 것 |
| `mock` | `mock_components/GenericSystem` | 로봇도 시뮬도 없이 계획만 |
| `real` | `fairino_hardware/FairinoHardwareInterface` | 실물. `robot_ip`만 준다 |

위쪽은 전부 그대로다. MoveIt, `joint_trajectory_controller`, `motion_server`,
`task_manager`는 `ros2_control` 인터페이스만 보므로 어느 쪽이 밑에 있는지
모른다. **이것이 이 프로젝트에서 가장 값나가는 성질이고, 이미 있다.**

```bash
ros2 launch box_cell_bringup demo.launch.py hardware:=real robot_ip:=192.168.58.2
ros2 launch box_cell_bringup real.launch.py robot_ip:=192.168.58.2   # 실물 전용 구성
```

`real.launch.py`는 Gazebo가 없다는 전제로 쓴 것이다. 정답지(`/sim/boxes`)도
`box_feeder`도 없고, 기본이 `autostart:=false`다(실물이 저 혼자 시작하면 안
된다). 카메라 드라이버와 IO 게이트웨이는 밖에서 온다.

## 2. 그대로 쓸 수 있는 것

| 노드 | 실물에서 | 왜 |
|---|---|---|
| `motion_server` | 그대로 | ros2_control 위에서만 논다 |
| `task_manager` | 그대로 | 상태 기계. 하드웨어를 모른다 |
| `pallet_manager` | 그대로 | 순수 계산(패커) |
| `mes_client` / `mes_server` | 그대로 | HTTP + SQLite |
| `label_reader` | 그대로 | `sensor_msgs/CompressedImage`만 받는다 |
| `pose_resolver` | 그대로 | `CameraInfo` + TF로 호모그래피를 만든다 |
| `stack_check` | 그대로 | 깊이 영상과 TF |
| `scene_publisher` | 그대로 | `/pallet/state`를 본다. 시뮬 정답지를 안 본다 |
| `twin_bridge` | `source:=real`만 | 스키마가 같다 (런치가 자동으로 채운다) |

`scene_publisher`가 `/sim/boxes`가 아니라 `/pallet/state`를 보게 해 둔 것이
여기서 값을 한다. 계획 씬이 시뮬레이터에 묶여 있지 않다.

## 3. 바꿔 끼워야 하는 것

세 개다. 전부 **시뮬레이터의 정답지(`/sim/boxes`)를 읽는** 노드다.

### 3-1. `gripper_driver`

지금은 gz `DetachableJoint`를 붙였다 뗀다. 실물은 ES45 흡착 밸브의
디지털 출력 하나다. Modbus/TCP나 로봇 컨트롤러의 DO를 때리면 된다.

`/gripper/vacuum` 서비스 인터페이스는 그대로 두고 구현만 바꾼다.
지금은 `mode:=real`이면 밸브 상태가 `std_msgs/Bool` 하나로 `/io/tool_do`에
나간다. 그 토픽을 실제 24 V 출력으로 옮기는 IO 게이트웨이는 배선과 함께
온다. 배선 전 첫 시운전은 `io_backend:=none`으로 아예 내지 않는다.

**주의할 것이 하나 있다.** 지금 이 노드는 `/sim/boxes`를 보고 "무엇을
잡았는지"를 정하는데, 실물에는 그 정보가 없다. ES45에는 피드백이 없다.
그래서 실물에서는 잡았는지를 **정지 센서(집어 갔으면 판독 자리가 빈다)와
C2 적재 확인**으로만 안다. 그 두 개가 이미 있는 것이 다행이다.

### 3-2. `conveyor_driver`

지금은 gz `TrackController`에 표면 속도를 준다. 실물은 인버터 기동/정지
신호와 광전 센서 입력이다. `/belt/command` 서비스는 그대로 둔다.
`stop_sensor_x`로 위치를 재는 부분이 **실물에서는 센서 한 개의 on/off**로
바뀐다. 오히려 단순해진다.

이것도 `mode:=real`로 갈라 넣었다. 기동/정지는 `/io/conveyor_run`으로 나가고
정지 센서는 `/io/photo_eye`로 받는다. 실물에는 "어느 박스인지"가 없으므로
`/conveyor/box_at_station`에는 고정 이름 하나가 나간다. `task_manager`는
이름의 내용이 아니라 같은 이름이 계속 보이는지만 보므로(헛집기 인터록)
그대로 성립한다.

### 3-3. `box_feeder`

실물에는 없다. 상류 라인이 그 일을 한다. 안 띄우면 된다.

## 4. 진짜 일거리 — 좌표

코드보다 이쪽이 오래 걸린다.

1. **핸드아이 캘리브레이션 (기획서 E3).**
   `cell.yaml`의 카메라 `xyz`/`rpy`는 지금 도면값이다. 실물에서는
   체스보드로 다시 잡아야 한다. 다행히 `pose_resolver`가 호모그래피를
   TF에서 **계산**하므로(`source: computed`), 캘리브레이션 결과를 URDF의
   카메라 조인트에 넣으면 판독 파이프라인이 알아서 따라온다.
   호모그래피를 손으로 적어 넣는 구조였으면 여기서 다시 만들어야 했다.

2. **로봇 베이스 원점.** `cell.yaml`의 `frame`이 상판 기준이다.
   실물 셀의 로봇 플랜지를 실측해 `table_top_height`와 로봇 조인트 원점을
   맞춘다. 이게 틀리면 나머지가 전부 그만큼 밀린다.

3. **팔레트/컨베이어 실측.** `cell.yaml` 한 파일이므로 고칠 곳은 명확하다.

## 5. 안전 — 시뮬에는 없고 실물에는 반드시 있어야 하는 것

지금 없다. 실물에 붙이기 전에 있어야 한다.

- **비상정지.** 캐비닛에 모양만 있고 배선이 없다. 실물에서는 로봇
  컨트롤러의 안전 입력에 직결한다. ROS를 거치면 안 된다.
- **속도/힘 제한.** `joint_limits.yaml`의 값은 계획용이다. 실물에서는
  컨트롤러 쪽 안전 한계를 따로 건다.
- **작업 영역 감시.** 사람이 들어오면 서야 한다. 라이트커튼이나 스캐너.
- **`joint_limits.yaml`의 j2/j4/j5 제한을 그대로 쓸 것.** 시뮬에서 자세
  계열을 가두려고 좁혀 둔 값인데, 실물에서도 같은 이유로 필요하다.
  이 울타리가 없으면 같은 목표점에 팔이 반대로 접혀 들어가는 해가 나온다.

## 6. 남은 위험

- **속도.** 시뮬은 계획대로 정확히 움직인다. 실물 FR5는 추종 오차가 있다.
  `gz_ros2_control` 실측으로 매끈한 궤적은 0.009 rad, OMPL의 꺾인 경로는
  0.15 rad에서 중단됐다. 실물은 그보다 나쁠 수 있으므로 처음에는
  `transit_scale`을 낮춰 시작한다.
- **흡착.** 시뮬 흡착은 실패하지 않는다(붙이면 붙는다). 실물 골판지는
  표면과 테이프에 따라 놓친다. C2 적재 확인이 그것을 잡는 자리다.
- **조명.** 시뮬에서 판독이 되는 것은 조명이 균일하기 때문이다.
  실물은 자동 노출이 있어 오히려 유리하지만, 정반사는 실물이 더 심하다.
  기획서 5.4대로 저각 조명을 y 양쪽에 두는 설계는 그대로 유효하다.

## 7. 권하는 순서

1. `hardware:=mock`으로 전체 파이프라인을 로봇 없이 한 번 돌린다.
2. 실물 로봇만 붙이고(`hardware:=real`) 그리퍼·컨베이어는 손으로 돌린다.
   `motion_server`의 여섯 구간이 실물에서 그대로 도는지 본다.
3. 캘리브레이션. 그다음 카메라를 붙인다.
4. 그리퍼, 컨베이어 드라이버를 실물 구현으로 바꾼다.
5. `dry_run_scorer`를 실물에서 그대로 돌려 시뮬 점수와 견준다.
   **이것이 트윈이 실제로 쓸모 있는 지점이다.** 두 점수가 벌어지는 항목이
   곧 시뮬이 틀린 곳이다.
