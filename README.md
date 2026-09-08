# 물류 로봇 데모 — 시뮬레이션

기획서 `물류로봇데모_기획서 2.pdf`의 셀을 ROS 2 Jazzy + Gazebo Harmonic으로
그대로 옮긴 것이다. 확정 제원, 도달 검증 수치, 적재 격자 공식, 카메라 배치,
상태 기계, MES 스키마가 전부 기획서에서 왔다.

로봇은 **FAIRINO FR5**(v6, `fr5c`나 `fr5l`이 아닌 기본 FR5), 툴은 **ES45 진공
그리퍼**. 카메라는 넷이다.

| ID | 위치 | 기종 | 역할 |
|----|------|------|------|
| C1 | 컨베이어 탑뷰 | **D435f** | 판독 전담. 이 한 대로 판독이 끝난다 |
| C2 | 팔레트 탑뷰 | **D455** | 적재 확인, 디팔레타이징 전 재확인, 적층 붕괴 감지 |
| C3 | 갠트리 기둥 중단 | 미정 | MES 기록 사진, 트윈/관람 화면 |
| C4 | 손목 그리퍼 위 | **D405** | 판독 실패 시 근접 재시도 |

C3는 기획서에도 기종이 비어 있고 지시에도 없어서 일반 RGB로 두었다.
`cell.yaml`의 `cameras.c3_scene.model` 한 줄만 바꾸면 확정 기종으로 갈린다.

---

## 지금 바로 확인할 수 있는 것

ROS도 Gazebo도 없이, 이 저장소만으로 돌아가는 검증이 셋 있다. 배치가 맞는지,
팔이 실제로 닿는지, QR이 정말 읽히는지를 시뮬레이터를 켜기 전에 답한다.

```bash
python3 tools/verify_layout.py
```

기획서 "배치와 도달 검증"의 수치를 `cell.yaml`에서 다시 만들어 낸다.
로봇 중심에서 판독 위치 **414**, 팔레트 1 **554**, 팔레트 2 **603**,
예외 통 **369**. 팔레트 여덟 모서리 **398 ~ 806**. 적재 격자의 블록 140,
사방 여유 30, 간극 10. 전부 일치한다. 카메라는 화각과 거리에서 지상
분해능을 뽑아 40 mm QR이 몇 픽셀로 찍히는지까지 낸다.

```bash
tools/xacro_expand.sh src/box_cell_description/urdf/box_cell_robot.urdf.xacro -o /tmp/robot.urdf
python3 tools/verify_reach.py /tmp/robot.urdf     # numpy 필요
```

펼친 URDF로 FR5의 FK/IK를 직접 풀어 26개 자세의 도달성을 확인한다.
석션 TCP 오프셋 220.5 mm, 관절 한계, 그리고 **팔꿈치가 상판 아래로 내려가지
않을 것**까지 조건에 넣는다. 마지막 조건이 없으면 IK는 도달한다고 답하지만
실물은 상판을 때린다.

```bash
python3 tools/make_labels.py                      # qrcode, pillow 필요
```

박스 8개의 QR 라벨 텍스처와 MES 시드 데이터를 만든다. 생성한 라벨을 C1이
실제로 보게 될 크기(97 px)로 줄이고 노이즈를 얹어 디코드해 보면, 8개 전부
코드와 네 모서리가 나온다. 판독 파이프라인이 성립한다는 뜻이다.

---

## 시나리오

기본은 **infeed**다. `cell.yaml`의 `scenario.mode`가 고른다.

**infeed** — 박스가 컨베이어를 타고 들어온다. 정지 센서에서 서면 C1이 판독하고,
로봇이 흡착으로 집어 팔레트에 **구석부터 하나씩** 쌓는다. 1층 네 자리를
채우고 2층으로 올라간다. 팔레트가 차면 반출한 것으로 보고 비운 뒤 다음
로트를 받는다. 박스를 벨트에 올리는 것은 상류 라인의 일이고 로봇은 관여하지
않는다.

적재 순서는 `cell.yaml`의 격자 공식이 정한다. 슬롯 0이 (col 0, row 0),
즉 팔레트의 한 모서리다.

```
  슬롯 0 → 1 → 2 → 3   1층 (구석에서 시작해 라스터로)
  슬롯 4 → 5 → 6 → 7   2층
```

**circulate** — 기획서 "순환"의 두 팔레트 방식. 팔레트 1이 차면 로봇이
하나씩 꺼내 컨베이어에 되올리고, 판독을 거쳐 팔레트 2에 쌓는다. 2가 차면
방향이 뒤집힌다. 보충 없이 무인 연속 운전을 보이기 위한 구성이다.

실제 셀에서는 상류 컨베이어가 박스를 보내 온다. 시뮬레이터는 그 상류를
만들지 않는다. 대기 박스를 셀 밖 바닥에 세워 두었다가 벨트 입구로 하나씩
올린다. 셀 경계 밖의 일은 셀의 관심사가 아니다.

---

## 실제로 돌려서 확인한 것

컨테이너를 굽고 두 모드로 띄워 봤다. 결과를 그대로 적는다.

**빌드** — 11개 패키지 전부 통과. rosdep도 "All required rosdeps installed
successfully".

**`hardware:=mock` (Gazebo 없이 MoveIt까지)** — 기획서 R2가 요구하는 모드다.
여기서 확인된 것 :

- `controller_manager`가 `mock_components/GenericSystem`을 물고 두 컨트롤러
  활성화
- `scene_publisher`가 고정 충돌체 **18개** 등록 (상판 4조각, 컨베이어와 레일,
  기어박스, 팔레트 2, 예외 통, 갠트리 기둥 2 · 보 · 팔 2, 카메라 2)
- `pallet_manager`가 P1=8/8, P2=0/8로 기동하고 **기록의 역순**으로 반출
  (#7 → #6 → #5 …). 2층부터 꺼내야 무너지지 않는다.
- `motion_server`가 아홉 구간을 완주 :
  APPROACH → DESCEND → GRASP → RETREAT → TRANSIT → PLACE → RELEASE → DONE
- **픽앤플레이스 1회 11.7 ~ 12.3초.** 기획서 목표는 한 사이클 20초다.
- `scene_publisher`가 반출에 맞춰 `stacked_p1_s7` … 를 순서대로 제거
- MES 서버 연결, 품목 조회, 멱등 이벤트 기록

WAIT_BOX에서 멈추는 것은 정상이다. mock에는 Gazebo가 없으니 컨베이어 물리도
없고, 박스가 정지 센서에 도달할 수 없다.

**`hardware:=gazebo headless:=true`** — Gazebo Harmonic 쪽 :

- 월드 로드, 로봇과 셀 구조물 스폰, `gz_ros2_control` 설정 성공
- 컨트롤러 두 개 활성화, `joint_trajectory_controller`가 궤적을 실제로 실행
- **박스 8개가 팔레트 1의 정확한 슬롯 좌표에 스폰** — box_1이
  (0.915, 0.175, 0.800), box_2가 (0.985, 0.175, 0.800). cell.yaml의 격자 공식
  그대로다.
- `/sim/boxes`가 각 박스를 `pallet: 1, slot: 0, code: AXO-0001`로 올바르게 분류
- 카메라 4대 센서와 브리지 기동

**infeed 시나리오 (GPU, 헤드리스)** — 실시간 계수 **1.00**에서 :

- 박스 8개가 셀 밖에 대기하고, `/feeder/next`로 하나씩 벨트 입구에 오른다
- 컨베이어가 물리로 밀어 정지 센서에 세운다. 목표 405 mm에 **418 mm** 정지
  (관성 13 mm. 실물 벨트도 이렇다)
- C1이 QR을 읽는다 : `AXO-0001`, 한 변 147 px, 디코드 여유 0.92
- 호모그래피가 픽셀을 mm로 : 중심 (411.7, 727.1) mm, 회전 +0.7도.
  **정답지 대비 오차 1.6 mm / 2.1 mm** — 흡착 여유(컵 30 mm, 박스 60 mm)에
  비하면 넉넉하다
- MES 조회 : `AXO-0001 = 무선 이어폰 (전자)`
- `pallet_manager`가 **P1 #0 (층 0, 구석)** 배정
- 석션 흡착 성공

### 실측 : 실시간 계수를 0.004에서 1.00으로

세 단계를 밟았고, 각 단계가 무엇을 고쳤는지 수치로 남는다.

| 상태 | 실시간 계수 | GPU |
|---|---|---|
| 처음 (소프트웨어 EGL, 카메라 4대 30 Hz) | 0.004 | 0 % |
| NVIDIA EGL ICD 주입 후 | 0.016 | 21 % |
| 카메라 갱신률을 실제 쓰임에 맞춘 뒤 | **1.00** | 43 % |

**첫 번째 : NVIDIA EGL ICD가 없었다.** `nvidia-container-toolkit`은 드라이버
라이브러리(`libEGL_nvidia.so`)는 컨테이너에 넣어 주는데 등록 파일
`/usr/share/glvnd/egl_vendor.d/10_nvidia.json`은 넣어 주지 않았다. libglvnd가
mesa만 발견해 소프트웨어 EGL로 떨어졌다. 이미지에 그 파일을 직접 넣어 해결했다.

**두 번째 : 카메라가 병목이었다.** 물리가 아니었다. 카메라를 1 Hz로 낮췄더니
실시간 계수가 즉시 1.00이 나왔다. 넷을 30 Hz로 돌리면 프레임 복사만으로
CPU가 포화된다. 그래서 `cell.yaml`에 `stream:` 블록을 두어 시뮬레이터가
실제로 렌더링할 것을 따로 정한다. `realsense.yaml`의 30 Hz는 하드웨어
제원이라 그대로 둔다.

낮춰도 되는 근거는 기획서 5.4에 있다. "판독은 C1 한 대로 완결하고 나머지는
확인과 기록을 맡는다." C1은 10 Hz에 깊이를 끄고(판독에 깊이를 쓰지 않는다),
C2/C3는 5 Hz, C4는 10 Hz다.

## 실행에 필요한 것

이 PC는 Ubuntu 26.04라 ROS 2 Jazzy 공식 바이너리가 없다(Jazzy는 24.04용이다).
그래서 시뮬레이션은 컨테이너로 돈다. 기획서 4절이 엣지를 Docker로 배포한다고
정해 둔 것과 같은 방식이다.

Docker는 설치 완료. **남은 것은 NVIDIA 컨테이너 런타임 하나뿐이고, 위에서
적었듯 이건 선택이 아니다.** Ubuntu 기본 저장소에는 없어서 NVIDIA 저장소를
추가해야 한다.

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
```

```bash
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

```bash
sudo apt update && sudo apt install -y nvidia-container-toolkit && sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker
```

그다음 :

```bash
./docker/build.sh      # 이미 한 번 성공했다. 소스만 바뀌면 1분.
./docker/run.sh        # 전체 데모
```

`docker/run.sh`가 NVIDIA 런타임을 자동으로 감지해 `--gpus all`을 붙인다.
없으면 경고를 내고 소프트웨어 렌더링으로 떨어진다.

---

## 실행

```bash
./docker/run.sh                                    # Gazebo + MoveIt + 전체 로직
./docker/run.sh ros2 launch box_cell_bringup demo.launch.py hardware:=mock
./docker/run.sh ros2 launch box_cell_bringup demo.launch.py rviz:=true
./docker/run.sh ros2 launch box_cell_bringup demo.launch.py autostart:=false
```

`hardware:=mock`은 Gazebo 없이 `mock_components/GenericSystem`으로 돈다.
기획서 R2의 "실물 없이 MoveIt2까지 전부 구동"이 이것이다. 궤적은 제대로
계산되고 실행만 흉내다. 노트북에서 로직을 고칠 때 쓴다.

`autostart:=false`면 상태 기계가 IDLE에서 기다린다. 시작 순간을 사람이
잡고 싶을 때 :

```bash
ros2 topic pub --once /cell/command std_msgs/msg/String "{data: start}"
```

MES 대시보드는 <http://localhost:8020>.

---

## 무엇이 어디에 있는가

기획서 "패키지 구성"과 같은 이름을 쓴다.

| 패키지 | 내용 |
|--------|------|
| `box_cell_description` | **`config/cell.yaml` — 모든 치수의 단일 원본.** URDF/xacro, FR5 메시 |
| `box_cell_common` | `cell_geometry.py`. 노드들이 cell.yaml을 읽는 통로 |
| `box_cell_msgs` | `LabelDetection`, `PickPlace.action`, `NextSlot.srv` 등 (기획서 5.7) |
| `box_cell_moveit_config` | SRDF, 기구학, Pilz/OMPL 설정 |
| `box_cell_sim` | Gazebo 월드, 박스 스폰, ros_gz 브리지 |
| `box_cell_motion` | `motion_server`(pick_place 액션), `gripper_driver`(석션), `scene_publisher` |
| `box_cell_perception` | `camera_node`, `label_reader`(QR), `pose_resolver`(호모그래피) |
| `box_cell_logic` | `task_manager`(상태 기계), `pallet_manager`(적재 기록), `mes_client` |
| `box_cell_conveyor` | `conveyor_driver`. 벨트 구동과 정지 센서 |
| `box_cell_mes` | FastAPI + SQLite MES 서버, 대시보드 |
| `box_cell_bringup` | 런치와 파라미터 |

### cell.yaml 하나만 고치면 된다

상판 1200×800×H750, 로봇 중심 (405, 311), 컨베이어 800×150(벨트 상면 상판
위 180), 팔레트 300각 두 장 (950, 210)과 (950, 570), 예외 통 (95, 110),
박스 60×60×40 여덟 개. 이 숫자들은 `cell.yaml` 한 곳에만 있다.

- URDF(xacro)가 `xacro.load_yaml`로 읽는다
- 노드들이 `cell_geometry.py`로 읽는다
- 검증 도구가 같은 파일로 기획서 수치를 재현한다

치수를 노드에 다시 적으면 언젠가 반드시 어긋나고, 어긋난 줄도 모른 채
데모 당일에 알게 된다.

---

## 실물을 얼마나 따라갔나

의도적으로 실물 그대로 둔 것들이다. 편하게 만들면 시뮬레이터에서만 되는
데모가 되기 때문이다.

**석션에 센서가 없다.** ES45는 24 V I/O 장치다. 통신도 센서도 없고, 잡았는지
되물을 방법이 없다. 그래서 `/gripper/vacuum`의 응답은 "명령을 냈다"는 뜻이지
"잡았다"는 뜻이 아니다. 상위 로직은 진공을 켜고 정해진 시간을 기다리는 것
말고 할 수 있는 게 없다. `gripper_driver`는 흡착 대상을 고를 때도 실물의
흡착 조건(컵에서 상면까지 거리, 축 정렬 각도)을 그대로 판정하고, 못 맞추면
아무것도 붙지 않는다. 실물에서 헛집는 것과 같다.

**컨베이어가 물리로 움직인다.** Gazebo에는 벨트 표면 속도를 흉내 내는 시스템이
없다. 박스를 텔레포트로 옮기면 간단하지만, 그 물체는 속도도 접촉도 거짓이라
석션이 잡는 순간이 실물과 달라진다. 대신 `conveyor_driver`가 박스 속도를 읽고
마찰 보상 + 속도 오차 비례로 힘을 실어 벨트 속도를 따라가게 한다.

**컨트롤러가 100 Hz다.** 실물 FR5 컨트롤러의 관절 갱신 주기에 맞췄다.
시뮬레이터만 250 Hz로 돌리면 실물에서 재현되지 않는 매끄러움이 나온다.

**MoveIt은 Planning Scene에 등록된 것만 안다.** 상판(받침 개구 400각을 피해
네 조각), 컨베이어, 팔레트 2장, 예외 통, 그리고 **카메라 갠트리 기둥 두 개**를
등록한다. 기둥이 로봇 도달 고리(외경 884) 안에 있어서 이건 선택이 아니다.
적재된 박스는 시뮬레이터의 정답지가 아니라 `pallet_manager`의 **적재 기록**을
보고 만든다. 실물에는 정답지가 없고 기록만 있기 때문이다.

**판독 실패가 정상 동작의 일부다.** 3회까지 재시도하고, 3회째는 C4 손목
카메라로 근접 재시도한다. 그래도 안 되면 예외 통으로 보낸다.

**MES가 죽어도 로봇이 서지 않는다.** `mes_client`가 로컬 큐에 쌓았다가
재전송한다. `event_id`가 멱등키라 중복 기록이 생기지 않는다.

### 시뮬레이터만의 사정

`/sim/boxes`는 정답지다. 실물에는 없다. 읽어도 되는 곳은 셋뿐이고 코드에
그렇게 적어 두었다 — `conveyor_driver`(힘을 실을 대상), `gripper_driver`
(석션이 붙일 대상), Dry Run 채점. 판독과 적재 로직은 이것을 보지 않는다.

---

## FAIRINO 공식 코드를 어떻게 썼나

| 출처 | 가져온 것 |
|------|-----------|
| `frcobot_ros2` / `fairino_description` | FR5 v6 기구학, 관성 텐서, 관절 한계, 링크 이름 |
| `frcobot_ros2` / `fairino5_v6_moveit2_config` | SRDF 충돌 해제 쌍, 기구학 설정의 출발점 |
| `fairino_gazebo` / `fr5v6_ros2_control` | 시각 메시(DAE)와 충돌 메시(STL) |

커밋은 `vendor/UPSTREAM.txt`에 박아 두었다.

바꾼 것이 둘 있다.

**Gazebo Classic → Harmonic.** `fairino_gazebo`는 Gazebo Classic + ROS 2
Humble이다(`gzserver`, `gazebo_ros2_control/GazeboSystem`). Classic은 2025년
1월로 지원이 끝났고 Jazzy용 패키지가 없다. 기구학과 메시만 가져오고
ros2_control 계층은 `gz_ros2_control/GazeboSimSystem`으로 옮겼다.

**effort를 정격으로.** `fairino_gazebo`는 피크 토크(332/63 N·m)를 넣어
두었는데 정격(150/28)을 쓴다. 시뮬레이션이 실물보다 세게 나오면 안 된다.

---

## 아직 안 된 것

정직하게 적는다.

- **한 사이클을 끝까지 본 적이 아직 없다.** 흡착까지 확인했고 그 앞의 모든
  단계가 통과하지만, 팔레트에 실제로 놓이는 것까지는 못 봤다. 마지막
  시험 중에 **GPU 드라이버가 물려서**(`Unable to determine the device handle
  for GPU0`) 더 못 돌렸다. 재부팅하면 풀린다.
- **GUI 모드는 아직 못 봤다.** 이 세션에는 화면이 없어 헤드리스로만 돌렸다.
  `./docker/run.sh`는 X11을 넘겨주도록 되어 있다.
- **Pilz PTP가 종종 충돌 경로를 낸다.** ValidateSolution이 걸러 내고 OMPL이
  받아 주므로 동작에는 지장이 없지만, 계획 시간이 낭비된다. 고정 웨이포인트를
  더 촘촘히 잡으면 줄어들 여지가 있다.
- **갠트리 기둥 위치는 잠정값이다.** 기획서 7절에서 "발주 도면에서 확정"으로
  남아 있다. `cell.yaml`의 `gantry:` 블록에 `tbd: true`로 표시해 두었고,
  도면이 나오면 그 블록만 고치면 형상과 Planning Scene이 함께 따라온다.
- **C3 기종 미정.** 위에 적은 대로 `model` 한 줄이다.
- **트윈 브리지(`twin_bridge`)는 아직 없다.** 기획서 D4 항목이다.
  `/cell/state`와 `/pallet/state`가 이미 나오고 있으므로 rosbridge를 얹으면
  된다.
- **Dry Run 채점기는 아직 없다.** 기획서 D3. `/sim/boxes`로 적재 결과를
  자동 채점하는 도구를 붙일 자리다.

## 다음에 볼 것

1. **재부팅.** 마지막 시험에서 GPU 드라이버가 물렸다. `nvidia-smi`가 GPU를
   다시 찾는지 확인하고 시작한다.
2. `./docker/run.sh` 로 전체 데모(GUI). 먼저 볼 것 :
   - 실시간 계수가 0.8 이상인가
   - 박스가 팔레트 구석부터 하나씩 쌓이는가
   - `/cell/state`의 `last_cycle_sec` — 기획서 목표는 20초
3. 여덟 개를 다 쌓은 뒤 반출과 재투입이 도는지. 그게 되면 무인 연속 운전이다.
4. 카메라 부하가 남으면 `cell.yaml`의 `cameras.*.stream.color_rate`를 더 낮춘다.
   `realsense.yaml`은 하드웨어 제원이므로 건드리지 않는다.
# fr5_gazebo
# fr5_gazebo
