# Ubuntu 24.04에 직접 깔기 (Docker 없이)

**결론부터.** 명령 셋이다.

```bash
./scripts/install_deps.sh     # ROS 2 Jazzy + Gazebo Harmonic + MoveIt (20~40분)
./scripts/build.sh            # 워크스페이스 빌드 (5~10분)
./scripts/run_demo.sh         # 전체 데모
```

막히면 `./scripts/doctor.sh`. 무엇이 빠졌는지 한 줄씩 답한다.

---

## 왜 컨테이너를 벗어나는가

컨테이너는 개발 PC가 Ubuntu 26.04였기 때문에 필요했다. ROS 2 Jazzy의 공식
바이너리는 24.04용뿐이라 26.04에는 얹을 수가 없었다. 대상 장비가 24.04면
그 이유가 통째로 사라진다. Jazzy도 Gazebo Harmonic도 공식 패키지가 있다.

그리고 컨테이너에서 실시간 계수를 0.004에서 1.00으로 끌어올리느라 했던 일의
절반은 **컨테이너였기 때문에 생긴 일**이었다.

| 컨테이너에서 필요했던 것 | 호스트에서는 |
|---|---|
| `/usr/share/glvnd/egl_vendor.d/10_nvidia.json`을 직접 만들어 넣기 | 드라이버 패키지가 이미 넣어 둔다 |
| `__GLX_VENDOR_LIBRARY_NAME=nvidia`로 벤더 못박기 | 세션이 직접 고른다 |
| `--gpus all`, `NVIDIA_DRIVER_CAPABILITIES=all` | 해당 없음 |
| `--shm-size=2g` (기본 64 MB로는 Gazebo가 죽는다) | 호스트 `/dev/shm`은 보통 메모리의 절반 |
| `--network host`로 DDS 디스커버리 살리기 | 해당 없음 |
| `-v /tmp/.X11-unix` 마운트와 `xhost +local:docker` | 해당 없음 |

남는 것은 **소켓 수신 버퍼** 하나다. 이건 호스트 커널 설정이라 컨테이너
안에서도 밖에서도 같다. `install_deps.sh`가 `/etc/sysctl.d/60-box-cell.conf`로
16 MB를 박아 둔다.

컨테이너를 지우지는 않았다. 26.04 개발 PC에서 확인할 일이 남아 있고,
기획서 4절이 엣지 배포를 Docker로 못박고 있다. `docker/` 아래는 그대로 있다.

---

## 무엇이 깔리는가

`scripts/install_deps.sh`의 apt 목록은 `docker/Dockerfile`의 것과 같다.
두 벌이 갈리지 않게 **스크립트 쪽을 원본으로 보고** Dockerfile을 따라오게 한다.

- `ros-jazzy-desktop`, `ros-jazzy-ros-gz`(Gazebo Harmonic을 끌어온다),
  `gz_ros2_control`, `ros2_control`/`ros2_controllers`, MoveIt 2와 Pilz/OMPL,
  `cv_bridge`, `rviz2`
- 파이썬 : `cv2 numpy yaml PIL qrcode pyzbar fastapi uvicorn`

파이썬은 **apt를 먼저 본다.** apt에 없는 것(`pyzbar`, `python-barcode`)만
`pip3 install --user`로 `~/.local`에 넣는다. 배포판 파이썬(`/usr/lib/python3`)은
건드리지 않는다. 컨테이너에서 쓰던 `--break-system-packages`를 호스트에
그대로 옮기면 언젠가 apt와 싸운다.

`pyzbar`는 `libzbar0`(apt)가 있어야 import된다. 스크립트가 함께 깐다.

---

## 셸마다 한 번

```bash
source scripts/setup_env.sh
```

`docker/entrypoint.sh`가 하던 일을 한다.

- `/opt/ros/jazzy/setup.bash`와 이 워크스페이스의 `install/setup.bash`
- `GZ_SIM_RESOURCE_PATH` — 메시와 라벨 텍스처를 gz가 찾는 경로
- `BOX_CELL_DATA_DIR` — 산출물이 쌓이는 곳. 기본 `/tmp/box_cell`
- 라벨 텍스처가 `cell.yaml`보다 오래됐으면 다시 만드는 함수
  (`box_cell_refresh_labels`)

`scripts/build.sh`와 `scripts/run_demo.sh`는 이 파일을 스스로 부른다.
직접 `ros2 launch`를 치고 싶을 때만 손으로 source하면 된다.

### 산출물 위치

적재 기록, MES DB와 큐, 트윈 JSON, Dry Run 점수, 판독 실패 프레임이 전부
`BOX_CELL_DATA_DIR`에 모인다. 기본값은 예전 그대로 `/tmp/box_cell`이다.

컨테이너에서는 그래도 됐다. 이미지 안이고, 컨테이너가 죽으면 같이 사라졌다.
호스트에서는 `/tmp`가 여러 사람 것이고 재부팅마다 지워진다. **실물 셀에서는
반드시 옮긴다.**

```bash
export BOX_CELL_DATA_DIR=/var/lib/box_cell
```

적재 기록이 재부팅으로 사라지면 로봇은 빈 팔레트라고 믿고 이미 박스가 있는
자리에 내려놓는다.

---

## 실행

```bash
./scripts/run_demo.sh                                  # 전체 데모(GUI)
./scripts/run_demo.sh headless:=true                   # 화면 없이. 카메라는 돈다
./scripts/run_demo.sh hardware:=mock                   # Gazebo 없이 MoveIt까지
./scripts/run_demo.sh rviz:=true autostart:=false      # 시작을 사람이 잡는다
./scripts/run_demo.sh hardware:=real robot_ip:=192.168.58.2   # 실물 (docs/digital_twin.md)
```

인자는 그대로 `demo.launch.py`로 넘어간다. `ros2 launch`를 직접 쳐도 같다.

```bash
source scripts/setup_env.sh
ros2 launch box_cell_bringup demo.launch.py headless:=true
```

MES 대시보드는 <http://localhost:8020>, 트윈 JSON은 <http://localhost:8030/twin>.

시작 신호를 직접 줄 때 :

```bash
ros2 topic pub --once /cell/command std_msgs/msg/String "{data: start}"
```

---

## GPU

호스트에서는 **따로 할 일이 없다.** 확인만 한다.

```bash
glxinfo -B | grep "OpenGL renderer"     # llvmpipe면 소프트웨어다
nvidia-smi                              # 데모 중에 사용률을 본다
```

`llvmpipe`가 나오면 드라이버 문제다. 컨테이너에서처럼 ICD 파일을 넣어
해결할 일이 아니다. `ubuntu-drivers devices`로 권장 드라이버를 확인하고
`sudo ubuntu-drivers autoinstall` 후 재부팅한다.

Wayland 세션이면 GUI가 Xwayland를 거친다. 컨테이너에서 GPU 사용률이 0으로
떨어지던 그 경로인데, 호스트에서는 보통 문제가 되지 않는다. 그래도 창 모드가
유난히 느리면 로그인 화면에서 Xorg 세션을 골라 한 번 견줘 볼 것.

GPU가 아예 없으면 돌기는 한다. 카메라 4대가 못 따라와 실시간 계수가 크게
떨어질 뿐이다. 그때는 `cell.yaml`의 `cameras.*.stream.color_rate`를 더 낮춘다.
`realsense.yaml`은 하드웨어 제원이므로 건드리지 않는다.

---

## mock 모드로 실제 확인한 것

호스트 설치가 막힌 환경(ROS apt 저장소가 방화벽에 걸리는 곳)에서 conda
(RoboStack `robostack-jazzy` 채널)로 같은 Jazzy를 깔아 한 번 끝까지 돌려
봤다. **apt 경로와 같은 환경은 아니다.** 아래는 코드가 도는지에 대한
확인이지 `install_deps.sh`가 도는지에 대한 확인이 아니다.

    11개 패키지 colcon 빌드      전부 통과
    controller_manager          mock_components/GenericSystem 적재, 100 Hz
    컨트롤러                     joint_state_broadcaster, joint_trajectory_controller 활성
    move_group                  OMPL + Pilz(PTP/LIN/CIRC) 적재, pick_ik 동작
    scene_publisher             고정 충돌체 18개 등록
    MES                         FastAPI :8020, 품목 8건 적재, mes_client 연결
    twin_bridge                 :8030/twin 에 JSON, source=sim
    verify_place_descent.py     자리 20개 전부 하강 100%,
                                실제 로트 6단계도 전부 100%
                                해가 전부 j4 -88.3~-84.6, j5 +90 계열에 들어온다
    /pick_place 액션             APPROACH->...->DONE 완주, 1사이클 13.7초
    twin_mirror                 /real/joint_states를 넣으니 mock 로봇이
                                그 값으로 정확히 따라왔다
    BOX_CELL_DATA_DIR           산출물 4종이 지정한 디렉터리에 쌓였다

`/feeder/next`가 없다며 WAIT_BOX에서 도는 것은 정상이다. mock에는 Gazebo가
없으니 컨베이어 물리도 없고 박스가 정지 센서에 도달할 수 없다.

이때 두 가지를 고쳤다.

1. `box_cell_description/CMakeLists.txt`가 없는 `launch` 디렉터리를 설치
   목록에 적고 있었다. `ament_cmake_symlink_install`은 없는 디렉터리에
   에러를 낸다(평범한 `install(DIRECTORY)`는 조용히 넘어간다). 새로 클론한
   워크스페이스의 **첫 빌드가 여기서 멈춘다.**
2. `pick_ik`는 conda 채널에 없어 소스로 빌드했다. apt에는
   `ros-jazzy-pick-ik`가 있으므로 24.04 호스트에서는 해당 없다.

---

## 자주 걸리는 것

**`ros2: command not found`** — `source scripts/setup_env.sh`를 안 했다.
셸을 새로 열 때마다 필요하다. 매번 치기 싫으면 `~/.bashrc`에 넣되,
ROS를 여러 버전 쓸 생각이면 넣지 않는 편이 낫다.

**`Package 'box_cell_bringup' not found`** — 빌드가 안 됐거나
`install/setup.bash`를 source하지 않았다. `./scripts/build.sh`.

**gz가 메시를 못 찾는다 (`Unable to find file with URI`)** —
`GZ_SIM_RESOURCE_PATH`가 비었다. `setup_env.sh`를 거치지 않고 `ros2 launch`를
직접 쳤을 때 그렇다.

**QR이 하나도 안 읽힌다** — 라벨 텍스처가 `cell.yaml`보다 오래됐을 수 있다.
`python3 tools/make_labels.py`로 다시 만든다. `run_demo.sh`는 자동으로 본다.

**영상 토픽이 뚝뚝 끊긴다** — `net.core.rmem_max`가 4 MB면 1920x1080 원본
컬러(6.2 MB)는 **한 장이 버퍼보다 커서** 대부분 유실된다. 판독기는 압축
토픽을 받으므로 데모는 돌지만, RViz에서 `image_raw`를 켜면 바로 티가 난다.
`install_deps.sh`가 16 MB로 올린다. 확인 : `sysctl net.core.rmem_max`.

**여러 대가 서로의 토픽을 본다** — 같은 망에 다른 ROS 2 PC가 있으면
`ROS_DOMAIN_ID`를 갈라 준다. 한 대 안에서만 돌릴 것이면
`export ROS_LOCALHOST_ONLY=1`.

**빌드가 메모리로 죽는다** — `colcon build --parallel-workers 2`.
MoveIt이 붙는 C++ 패키지가 한 번에 여러 개 돌면 8 GB로는 모자랄 수 있다.

**`error: option --editable not recognized`** — setuptools 80 이상에서
`colcon build --symlink-install`이 깨진다(`setup.py develop`이 없어졌다).
24.04의 apt 파이썬은 68이라 해당 없지만, pip로 setuptools를 올렸으면
`pip3 install --user "setuptools<80"`으로 되돌리거나 `--symlink-install`을
빼고 빌드한다.

---

## 검증 도구도 그대로 돈다

ROS 없이 돌아가는 셋은 예전과 같다.

```bash
python3 tools/verify_layout.py
tools/xacro_expand.sh src/box_cell_description/urdf/box_cell_robot.urdf.xacro -o /tmp/robot.urdf
python3 tools/verify_reach.py /tmp/robot.urdf
python3 tools/verify_camera_clearance.py /tmp/robot.urdf
```

`xacro_expand.sh`는 이제 **호스트의 xacro를 먼저 쓴다.** 없으면 예전처럼
컨테이너로 떨어진다. 호스트 xacro가 `$(find box_cell_description)`을 풀려면
빌드가 한 번은 되어 있어야 한다.

시뮬레이터를 띄운 뒤 도는 둘도 같다.

```bash
python3 tools/verify_place_descent.py
python3 tools/measure_read_rate.py 30
```
