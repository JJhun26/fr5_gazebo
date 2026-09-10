# 디지털 트윈 — 실물 FR5와 잇는 자리

이 문서는 **아직 하지 않은 일**에 대한 것이다. 지금 저장소에 들어 있는 것은
실물을 붙일 자리와 그 자리의 계약이고, 실물은 아직 없다. 무엇이 준비돼
있고 무엇이 비어 있는지를 섞이지 않게 적는다.

먼저 읽을 것 : `docs/real_robot_bringup.md`(무엇을 바꿔 끼워야 하는가),
`docs/native_ubuntu24.md`(호스트에 까는 법).

---

## 트윈이 무엇이어야 하는가

"시뮬레이터가 실물처럼 생겼다"는 트윈이 아니다. 쓸모 있으려면 셋이 필요하다.

1. **같은 코드가 양쪽에서 돈다.** 시뮬에서 고친 로직이 실물에서 그대로
   돌지 않으면, 시뮬에서 한 검증은 시뮬에 대한 검증일 뿐이다.
2. **같은 스키마로 상태가 나온다.** 두 쪽을 같은 화면에서 견줄 수 있어야
   한다. 견줄 수 없으면 어디가 다른지도 모른다.
3. **다른 곳이 수치로 나온다.** 트윈의 값어치는 닮음이 아니라 **차이**에
   있다. 같은 지표를 양쪽에서 재서 벌어지는 항목이 곧 시뮬이 틀린 곳이다.

1번은 이미 되어 있다. 2번은 절반. 3번은 도구는 있고 실물 데이터가 없다.

---

## 지금 있는 것

### 1. 하드웨어 계층 교체 (완성)

`box_cell_robot.urdf.xacro`의 인자 하나가 `ros2_control` 플러그인을 고른다.

| `hardware` | 플러그인 | 무엇이 도는가 |
|---|---|---|
| `gazebo` | `gz_ros2_control/GazeboSimSystem` | Gazebo Harmonic 안의 로봇 |
| `mock` | `mock_components/GenericSystem` | 로봇도 시뮬도 없이 계획만 |
| `real` | `fairino_hardware/FairinoHardwareInterface` | 실물 FR5. `robot_ip`만 준다 |

위쪽(MoveIt, `joint_trajectory_controller`, `motion_server`, `task_manager`)은
`ros2_control` 인터페이스만 보므로 밑에 무엇이 있는지 모른다.

```bash
ros2 launch box_cell_bringup demo.launch.py hardware:=real robot_ip:=192.168.58.2
ros2 launch box_cell_bringup real.launch.py robot_ip:=192.168.58.2   # 실물 전용 구성
```

`real.launch.py`가 따로 있는 이유는 실물에만 있는 사정 때문이다. Gazebo가
없으면 정답지(`/sim/boxes`)도 상류 라인 역할(`box_feeder`)도 없다. 그 자리에
무엇이 들어와야 하는지가 그 파일의 인자와 주석이다. 그리고 기본이
`autostart:=false`다. 실물이 사람 없이 저 혼자 시작하면 안 된다.

`fairino_hardware`는 이 저장소에 없다. FAIRINO의 `frcobot_ros2`에서 온다
(`vendor/UPSTREAM.txt`의 커밋). 실물을 붙일 때 워크스페이스에 함께 빌드한다.

### 2. 주변 장치의 I/O 계약 (자리만)

로봇은 드라이버 하나로 갈리지만, 주변 장치는 그렇지 않다. 그래서 노드가
직접 장치를 때리지 않고 **토픽 한 겹**을 두었다. 이 토픽을 실제 24 V I/O로
옮기는 것이 IO 게이트웨이 노드의 일이고, 그 노드는 배선과 함께 온다.

| 노드 | sim에서 | real에서 내는/받는 것 | 위쪽이 보는 것(불변) |
|---|---|---|---|
| `gripper_driver` | gz `DetachableJoint` | `/io/tool_do` (Bool, 밸브) | `/gripper/vacuum` 서비스 |
| `conveyor_driver` | 롤러 각속도 `/conveyor/belt_speed` | `/io/conveyor_run` (Bool, 인버터)<br>`/io/photo_eye` (Bool, 정지 센서) | `/belt/command` 서비스,<br>`/conveyor/*` 상태 토픽 |

두 노드 다 `mode:=sim|real` 파라미터 하나로 갈린다. `demo.launch.py`는
`hardware`에서 자동으로 정한다.

**왜 컨트롤러 서비스를 직접 부르지 않는가.** 부르는 순간 노드가 특정 컨트롤러
모델의 서비스 이름과 타입에 묶인다. 밸브를 로봇 컨트롤러의 툴 DO에 물릴지
별도 Modbus 모듈에 물릴지는 아직 정해지지 않았다(기획서 E 항목, 하드웨어
납기 의존). 토픽 한 겹을 두면 그 결정이 이 파일들 밖으로 나간다.

배선 전 첫 시운전은 `io_backend:=none`으로 시작한다. 신호를 아예 내지 않고
팔만 움직여 본다.

**실물에는 "어느 박스인지"가 없다.** 광전 센서는 있다/없다만 안다. 그래서
`conveyor_driver`가 real 모드에서는 고정 이름(`box_at_station`) 하나를
`/conveyor/box_at_station`에 낸다. `task_manager`는 이름의 내용이 아니라
**같은 이름이 계속 보이는지**만 본다(집어 갔으면 자리가 빈다는 헛집기
인터록). 그래서 이 대체가 성립한다.

### 3. 상태를 밖으로 (`twin_bridge`, 완성)

셀 상태를 JSON 한 덩어리로 네 곳에 낸다 — 파일(`$BOX_CELL_DATA_DIR/twin.json`),
토픽(`/twin/state`), HTTP(`:8030/twin`), WebSocket(`:8030/ws`).

스키마가 시뮬/실물에 의존하지 않는다. `source` 항목만 `sim`/`real`로 갈린다.
`demo.launch.py`와 `real.launch.py`가 `hardware`에 맞춰 자동으로 채운다.

내기만 하고 받지 않는다. 트윈이 셀을 조종하면 그것은 트윈이 아니라 검증되지
않은 제2의 상위 제어기다.

### 4. 상태를 안으로 (`twin_mirror`, 새로 넣음 · 실물로 확인 안 됨)

반대 방향이다. 실물 셀의 관절 상태를 받아 시뮬레이터의 로봇을 따라 움직이게
한다. 실물을 보면서 시뮬 화면으로 같은 장면을 보는, 흔히 말하는 그 트윈이다.

```bash
# 실물 셀 쪽 (또는 실물 데이터가 /real로 넘어오는 PC)
ros2 launch box_cell_bringup real.launch.py robot_ip:=192.168.58.2

# 트윈 쪽. 상태 기계는 세워 둔다.
ros2 launch box_cell_bringup demo.launch.py twin:=true autostart:=false
```

동작은 이렇다.

```
실물 /real/joint_states  ->  twin_mirror  ->  /joint_trajectory_controller/joint_trajectory
                                              (lookahead 0.15초짜리 한 점)
```

**왜 관절을 직접 쓰지 않는가.** gz_ros2_control에는 관절을 텔레포트시키는
창구가 없고, 있어도 쓰면 안 된다. 텔레포트하면 속도가 거짓이 되고 접촉이
튄다. 그 화면은 실물과 닮았을 뿐 물리가 끊겨서, 붙잡고 있는 박스가 어떻게
될지는 실물과 무관해진다. 컨트롤러로 밀어 넣으면 시뮬 로봇은 실물과 같은
경로로 움직인다.

**주의 두 가지.**
- 시뮬 쪽 `task_manager`가 같이 돌면 둘이 같은 컨트롤러를 두고 다툰다.
  미러로 쓸 때는 `autostart:=false`로 세워 둔다.
- 실물 소식이 `timeout`(기본 1초) 넘게 끊기면 밀어 넣기를 멈춘다. 실물이
  죽었는데 시뮬이 마지막 자세로 계속 명령받고 있으면 화면이 거짓말을 한다.

**아직 실물로 확인하지 않았다.** 실물 관절 상태가 들어오는 토픽 이름과
관절 이름 규약(`j1`..`j6`)이 맞는지가 첫 확인 항목이다. 다르면 `source_topic`과
`joints` 파라미터로 맞춘다.

두 대가 다른 망이면 토픽을 넘겨 주는 것이 따로 필요하다(`domain_bridge`,
또는 DDS/zenoh 라우터). `twin_mirror`는 토픽 이름만 본다. 어느 경로로 왔는지
모른다.

### 5. 차이를 재는 자 (`dry_run_scorer`, 완성 · 실물 데이터 없음)

처리량, 사이클 시간, 판독 성공률과 신뢰도, 예외 사유, 적재 정확도를
`$BOX_CELL_DATA_DIR/dry_run.json`에 쓴다. **실물에서 그대로 돌린다.**
두 점수가 벌어지는 항목이 곧 시뮬이 틀린 곳이다.

적재 정확도만 시뮬 정답지를 보므로 실물에서는 그 항목이 비고, 대신 C2
적재 확인(`stack_check`)의 수치가 남는다.

---

## 아직 없는 것

정직하게 적는다. 순서는 중요한 것부터다.

1. **IO 게이트웨이 노드.** `/io/tool_do`, `/io/conveyor_run`, `/io/photo_eye`를
   실제 24 V I/O로 옮기는 노드. 배선 방식이 정해지면 30줄짜리다. 지금은
   토픽만 나가고 아무 데도 닿지 않는다.
2. **핸드아이 캘리브레이션.** `cell.yaml`의 카메라 `xyz`/`rpy`는 도면값이다.
   실물에서는 체스보드로 다시 잡아 URDF의 카메라 조인트에 넣는다.
   `pose_resolver`가 호모그래피를 TF에서 **계산**하므로 그것만으로 판독
   파이프라인이 따라온다. 손으로 적어 넣는 구조였으면 여기서 다시 만들어야
   했다. `docs/real_robot_bringup.md` 4절.
3. **카메라 드라이버 배선.** `realsense2_camera`를 띄우고 토픽 이름을 시뮬과
   맞춰 remap한다(`/c1_conveyor/color/image/compressed` 등). 이름만 맞으면
   인식 노드는 그대로 돈다. `real.launch.py`는 이것을 띄우지 않는다.
4. **안전.** 비상정지, 라이트커튼, 속도/힘 제한. **ROS를 거치지 않는다.**
   컨트롤러의 안전 입력에 직결한다. 이건 협상 대상이 아니다.
   `docs/real_robot_bringup.md` 5절.
5. **역방향 상태(실물 -> 시뮬)의 나머지.** `twin_mirror`는 지금 관절과
   (선택적으로) 적재 기록만 비춘다. 실물 박스가 어디 있는지는 비추지 않는다.
   비추려면 C2 깊이에서 박스 자세를 뽑아 시뮬 월드에 세워야 하는데,
   그건 인식 쪽 새 일감이지 브리지의 일이 아니다.

---

## 권하는 순서

`docs/real_robot_bringup.md` 7절과 같다. 여기서는 명령까지 적는다.

```bash
# 1. 로봇 없이 전체 파이프라인
./scripts/run_demo.sh hardware:=mock

# 2. 실물 로봇만. 그리퍼/컨베이어는 손으로 돌린다.
#    io_backend:=none이라 신호를 아예 내지 않는다. 팔만 본다.
ros2 launch box_cell_bringup real.launch.py \
    robot_ip:=192.168.58.2 perception:=false io_backend:=none autostart:=false

# 3. 캘리브레이션 후 카메라를 붙인다
ros2 launch box_cell_bringup real.launch.py robot_ip:=192.168.58.2 autostart:=false

# 4. IO 게이트웨이를 띄우고 io_backend:=topic으로 그리퍼/컨베이어를 실물에
ros2 launch box_cell_bringup real.launch.py robot_ip:=192.168.58.2

# 5. 트윈 화면. 실물을 시뮬로 비추면서 두 점수를 견준다
./scripts/run_demo.sh twin:=true autostart:=false
diff <(jq . $BOX_CELL_DATA_DIR/dry_run.json) <(curl -s http://<실물PC>:8030/twin | jq .)
```

2번에서 처음 팔을 움직일 때는 `transit_scale`을 낮춰 시작한다. 시뮬은 계획
대로 정확히 움직이지만 실물 FR5는 추종 오차가 있다. `gz_ros2_control` 실측으로
매끈한 궤적은 0.009 rad, OMPL의 꺾인 경로는 0.15 rad에서 중단됐다. 실물은
그보다 나쁠 수 있다.

`joint_limits.yaml`의 j2/j4/j5 제한은 **실물에서도 그대로 쓴다.** 시뮬에서
자세 계열을 가두려고 좁혀 둔 값인데, 같은 이유가 실물에도 있다. 이 울타리가
없으면 같은 목표점에 팔이 반대로 접혀 들어가는 해가 나온다.
