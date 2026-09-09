#!/usr/bin/env python3
"""셀 전체의 상태 기계. 기획서 5절 "상태 기계".

    IDLE -> WAIT_BOX -> STOPPED -> READ -> QUERY -> PLAN -> EXECUTE -> RECORD -> IDLE
    READ 실패 3회 -> EXCEPTION -> 예외 통 이송 -> IDLE
    RECORD에서 팔레트 가득 -> DEPALLETIZE -> 반출 후 대상 팔레트 전환 -> IDLE

전이를 갖는 노드는 이것 하나다. 나머지는 전부 이 노드가 부르는 액션과
서비스일 뿐이고, 스스로 판단하지 않는다. 데모 중 무엇이 왜 멈췄는지
한 군데만 보면 되게 하려는 것이다.

시나리오가 둘이다. cell.yaml의 scenario.mode가 고른다.

  infeed (기본)
    박스가 컨베이어를 타고 들어온다. 정지 센서에서 서면 판독하고, 로봇이
    흡착으로 집어 팔레트에 구석부터 하나씩 쌓는다. 팔레트가 차면 반출한
    것으로 보고 비운 뒤 다음 로트를 받는다. 박스를 벨트에 올리는 것은
    상류 라인(시뮬레이터에서는 box_feeder)의 일이고 로봇은 관여하지 않는다.

  circulate
    기획서 "순환"의 두 팔레트 방식. 팔레트 1이 차면 로봇이 하나씩 꺼내
    컨베이어에 되올리고, 판독을 거쳐 팔레트 2에 쌓는다. 2가 차면 방향이
    뒤집힌다. 보충 없이 무인 연속 운전을 보이기 위한 구성이라, 기동 직후
    첫 동작이 판독이 아니라 디팔레타이징이다.

상태 기계는 별도 스레드에서 돈다. 픽앤플레이스 한 번이 십수 초라 콜백
안에서 기다릴 수 없기 때문이다. 노드는 MultiThreadedExecutor로 돌린다.
"""

from __future__ import annotations

import threading
import time

import rclpy
from box_cell_common.cell_geometry import CellGeometry
from box_cell_msgs.action import PickPlace
from box_cell_msgs.msg import CellState, LabelDetection, PalletState
from box_cell_msgs.srv import BeltCommand, ItemQuery, NextSlot, ReleaseSlot
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

S = CellState


def down_pose(x: float, y: float, z: float, yaw: float = 0.0) -> Pose:
    """TCP 수직 하향. rpy (pi, 0, yaw)를 쿼터니언으로."""
    import math

    half = yaw / 2.0
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x = math.cos(half)
    p.orientation.y = math.sin(half)
    p.orientation.z = 0.0
    p.orientation.w = 0.0
    return p


class TaskManager(Node):
    def __init__(self) -> None:
        super().__init__("task_manager")

        # 벨트 정지 후 박스가 멎기를 기다리는 시간
        # 예외 이송을 몇 번까지 다시 해 볼지. 넘으면 잼으로 본다.
        # 집은 뒤 정지 센서를 다시 보기까지 기다리는 시간
        self.declare_parameter("pick_check_sec", 1.0)
        self.declare_parameter("jam_retries", 2)
        self.declare_parameter("settle_time", 0.6)
        self.declare_parameter("read_retries", 3)      # 기획서 : 3회 실패면 예외 통
        self.declare_parameter("read_timeout", 3.0)
        self.declare_parameter("belt_timeout", 25.0)
        self.declare_parameter("target_pallet", 0)     # 0이면 cell.yaml을 따른다
        self.declare_parameter("source_pallet", 1)     # circulate에서 꺼내는 쪽
        self.declare_parameter("auto_start", True)
        self.declare_parameter("use_mes", True)

        self.cell = CellGeometry()
        self.cb = ReentrantCallbackGroup()

        self.state = S.IDLE
        self.detail = "대기"
        self.cycles = 0
        self.exceptions = 0
        self.last_cycle = 0.0
        self.mode = self.cell.scenario_mode
        override = int(self.get_parameter("target_pallet").value)
        self.target = override or int(self.cell.scenario.get("target_pallet", 2))
        self.source = int(self.get_parameter("source_pallet").value)
        if self.mode == "infeed":
            # 꺼내는 쪽이 없다. 박스는 상류에서 온다.
            self.source = 0
        self.running = bool(self.get_parameter("auto_start").value)
        # 순환에서 연속으로 빈 팔레트를 만난 횟수. 두 장 다 비면 멈춘다.
        self._empty_swaps = 0

        self.station_box = ""
        # 잼 관리. 아래 _to_exception / _jam 주석 참고.
        self.jam_tries: dict[str, int] = {}
        self.jammed = ""
        self.label: LabelDetection | None = None
        self._label_event = threading.Event()
        self.pallets: dict[int, PalletState] = {}

        # ------------------------------------------------------------ 연결
        self.pick_place = ActionClient(self, PickPlace, "/pick_place", callback_group=self.cb)
        self.belt = self.create_client(BeltCommand, "/belt/command", callback_group=self.cb)
        self.next_slot = self.create_client(NextSlot, "/pallet/next_slot", callback_group=self.cb)
        self.release = self.create_client(ReleaseSlot, "/pallet/release", callback_group=self.cb)
        self.read_c1 = self.create_client(
            Trigger, "/perception/c1_conveyor/read", callback_group=self.cb
        )
        self.read_c4 = self.create_client(
            Trigger, "/perception/c4_wrist/read", callback_group=self.cb
        )
        self.capture = self.create_client(
            Trigger, "/camera/c3_scene/capture", callback_group=self.cb
        )
        self.mes_item = self.create_client(ItemQuery, "/mes/item", callback_group=self.cb)
        # 상류 라인. infeed 모드에서 박스를 벨트에 올려 준다.
        self.c4_pose = self.create_client(
            Trigger, "/motion/c4_read_pose", callback_group=self.cb)
        self.go_home = self.create_client(Trigger, "/motion/home", callback_group=self.cb)
        self.feed_next = self.create_client(Trigger, "/feeder/next", callback_group=self.cb)
        self.feed_recycle = self.create_client(Trigger, "/feeder/recycle", callback_group=self.cb)

        self.commit_pub = self.create_publisher(String, "/pallet/commit", 10)
        self.event_pub = self.create_publisher(String, "/mes/event", 10)
        self.state_pub = self.create_publisher(CellState, "/cell/state", 10)

        self.create_subscription(String, "/conveyor/box_at_station", self._on_station, 10)
        self.create_subscription(LabelDetection, "/perception/label", self._on_label, 10)
        self.create_subscription(PalletState, "/pallet/state", self._on_pallet, 10)
        self.create_subscription(String, "/cell/command", self._on_command, 10)
        # 사람이 잼을 치운 뒤 라인을 다시 돌린다.
        self.create_service(Trigger, "/demo/resume", self._on_resume, callback_group=self.cb)

        self.create_timer(0.2, self._publish_state, callback_group=self.cb)
        self.worker = threading.Thread(target=self._loop, daemon=True)

    # ------------------------------------------------------------------ 입력
    def _on_station(self, msg: String) -> None:
        self.station_box = msg.data

    def _on_label(self, msg: LabelDetection) -> None:
        self.label = msg
        self._label_event.set()

    def _on_pallet(self, msg: PalletState) -> None:
        self.pallets[msg.pallet_id] = msg

    def _on_command(self, msg: String) -> None:
        cmd = msg.data.strip().lower()
        if cmd == "start":
            self.running = True
            self.get_logger().info("운전 시작")
        elif cmd == "stop":
            self.running = False
            self.get_logger().info("운전 정지 요청. 진행 중인 사이클은 끝까지 간다.")
        else:
            self.get_logger().warn(f"모르는 명령 : {msg.data}")

    # ------------------------------------------------------------------ 상태
    def _set(self, state: int, detail: str) -> None:
        self.state = state
        self.detail = detail
        self.get_logger().info(f"[{self._name(state)}] {detail}")

    @staticmethod
    def _name(state: int) -> str:
        return {
            S.IDLE: "IDLE", S.WAIT_BOX: "WAIT_BOX", S.STOPPED: "STOPPED",
            S.READ: "READ", S.QUERY: "QUERY", S.PLAN: "PLAN", S.EXECUTE: "EXECUTE",
            S.RECORD: "RECORD", S.EXCEPTION: "EXCEPTION", S.DEPALLETIZE: "DEPALLETIZE",
        }.get(state, str(state))

    def _publish_state(self) -> None:
        msg = CellState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.state
        msg.state_name = self._name(self.state)
        msg.detail = self.detail
        msg.source_pallet = self.source
        msg.target_pallet = self.target
        msg.cycle_count = self.cycles
        msg.last_cycle_sec = self.last_cycle
        msg.exception_count = self.exceptions
        self.state_pub.publish(msg)

    def _event(self, kind: str, code: str, extra: str = "") -> None:
        """MES 이벤트 한 줄. mes_client가 받아 서버로 올린다.

        여기서 직접 HTTP를 때리지 않는 이유는 기획서 4절의 하이브리드 구조다.
        정본은 서버에 있고, 통신이 끊기면 엣지의 mes_buffer가 쌓았다가
        재전송한다. task_manager는 그 사정을 몰라야 한다.
        """
        self.event_pub.publish(String(data=f"{kind}|{code}|{extra}"))

    # ---------------------------------------------------------------- 도우미
    def _belt(self, command: int) -> bool:
        req = BeltCommand.Request()
        req.command = command
        res = self.belt.call(req)
        return bool(res and res.accepted)

    def _photo(self) -> str:
        if not self.capture.service_is_ready():
            return ""
        res = self.capture.call(Trigger.Request())
        return res.message if res and res.success else ""

    def _wait_station(self, timeout: float) -> str:
        end = time.time() + timeout
        while time.time() < end:
            if self.station_box:
                return self.station_box
            time.sleep(0.1)
        return ""

    def _still_at_station(self, box_name: str) -> bool:
        """집어 간 뒤에도 그 박스가 정지 센서에 남아 있는가.

        센서 신호가 갱신될 시간을 조금 준다. 로봇이 박스를 들어 올린 직후에는
        conveyor_driver가 아직 옛 상태를 들고 있을 수 있다.
        """
        end = time.time() + float(self.get_parameter("pick_check_sec").value)
        while time.time() < end:
            if self.station_box != box_name:
                return False
            time.sleep(0.1)
        return self.station_box == box_name

    def _read_label(self, attempt: int) -> LabelDetection | None:
        """판독 한 번. 3회째는 C4 손목 카메라로 근접 재시도한다(기획서 5.4)."""
        client = self.read_c1
        who = "C1"
        close_up = False
        if attempt >= 3 and self.read_c4.service_is_ready():
            # 기획서 5.4의 근접 재시도. 셔터만 누르면 안 된다. 팔이 대기
            # 자세에 있으면 손목 카메라 화각에 컨베이어가 아예 없어서 3회째는
            # 언제나 실패한다. 손목을 판독 자리 위로 옮긴 다음에 찍는다.
            if self.c4_pose.service_is_ready():
                r = self.c4_pose.call(Trigger.Request())
                if r and r.success:
                    close_up = True
                else:
                    self.get_logger().warn(
                        f"C4 근접 자세로 못 갔다 : {r.message if r else '무응답'}. "
                        "C1으로 한 번 더 본다."
                    )
            if close_up:
                client = self.read_c4
                who = "C4 손목"
        self._label_event.clear()
        self.label = None
        res = client.call(Trigger.Request())

        def go_back() -> None:
            """근접 자세에서 대기 자세로 돌아온다.

            돌아오지 않으면 다음 동작이 검증하지 않은 자세에서 시작한다.
            그 자리에서 이어서 하면 적재 하강이 안 풀린다(joint_limits.yaml 주석).
            """
            if close_up and self.go_home.service_is_ready():
                self.go_home.call(Trigger.Request())

        if not (res and res.success):
            self.get_logger().warn(f"{who} 판독 실패 ({attempt}회) : {res.message if res else '무응답'}")
            go_back()
            return None
        if not self._label_event.wait(float(self.get_parameter("read_timeout").value)):
            self.get_logger().warn("pose_resolver 응답이 없다")
            go_back()
            return None
        ok = bool(self.label and self.label.ok and self._sane(self.label, who))
        go_back()
        return self.label if ok else None

    def _sane(self, label: LabelDetection, who: str) -> bool:
        """판독 좌표가 믿을 만한지 본다.

        스토퍼에 걸린 박스는 정지 센서 부근에 서 있다. 그보다 크게 벗어난
        좌표가 나왔다면 디코드는 됐어도 그 위치는 틀렸다. 옛 영상이거나 다른
        물체를 본 것이다. 실물 팔레타이저도 같은 검사를 한다.

        검사를 두 겹으로 둔다.
          창 검사   판독 위치가 정지 센서에서 accept_window 안에 있는가
          하한 검사 그 자리를 집으러 가면 팔이 C1 카메라를 때리지 않는가

        하한 검사가 따로 있는 이유는 방향이 비대칭이기 때문이다. 상류로
        벗어나면 카메라 밑을 파고들어 팔이 부딪히고, 하류로 벗어나면 그냥
        조금 멀 뿐이다. 근거 숫자는 tools/verify_camera_clearance.py가 낸다.

        틀린 판독은 실패로 세어 다시 찍는다. 3회를 못 넘기면 예외 통으로
        간다. 아무것도 없는 자리로 내려가는 것보다 낫다.
        """
        cx, cy = self.cell.read_station_xy()
        win = float(self.cell.data["read_station"]["accept_window"])
        x_min = float(self.cell.data["read_station"]["pick_x_min"])

        dx, dy = label.center_x - cx, label.center_y - cy
        if abs(dx) > win or abs(dy) > win:
            self.get_logger().warn(
                f"{who} 판독 위치가 판독 자리에서 "
                f"({dx*1000:+.0f}, {dy*1000:+.0f}) mm 벗어났다 "
                f"(허용 +-{win*1000:.0f}). 다시 찍는다."
            )
            return False
        if label.center_x < x_min:
            self.get_logger().warn(
                f"{who} 판독 위치 x={label.center_x*1000:.0f} mm는 하한 "
                f"{x_min*1000:.0f} mm보다 상류다. 집으러 가면 C1 카메라를 "
                "때린다. 다시 찍는다."
            )
            return False
        return True

    def _pick_place(self, pick: Pose, place: Pose, obj: str, home: bool = True,
                    box_size: list[float] | None = None) -> bool:
        goal = PickPlace.Goal()
        goal.pick_pose = pick
        goal.place_pose = place
        # Planning Scene에 붙일 상자 크기. MES에서 온 값이다.
        goal.box_size = [float(v) for v in (box_size or self.cell.default_box_size)]
        goal.approach_height = self.cell.approach_height
        goal.pick_object = obj
        goal.return_home = home
        # 동기 send_goal()은 goal handle이 아니라 GetResult 응답을 돌려준다.
        try:
            response = self.pick_place.send_goal(goal)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"/pick_place 호출 실패 : {exc}")
            return False
        if response is None:
            self.get_logger().error("motion_server 무응답")
            return False
        result = response.result
        if not result.success:
            self.get_logger().error(f"픽앤플레이스 실패 : {result.msg}")
        return bool(result.success)

    # ------------------------------------------------------------- 상태 기계
    def _loop(self) -> None:
        self.get_logger().info("연결을 기다린다")
        self.pick_place.wait_for_server()
        for cli in (self.belt, self.next_slot, self.release, self.read_c1):
            cli.wait_for_service()
        self.get_logger().info("연결 완료. 운전을 시작한다.")

        while rclpy.ok():
            if not self.running:
                self._set(S.IDLE, "정지 상태. /cell/command 로 start")
                time.sleep(0.5)
                continue
            if self.jammed:
                # 잼은 사람이 치워야 풀린다. 로봇이 계속 찔러 보게 두지 않는다.
                self._set(S.IDLE, f"잼 대기 : {self.jammed}. /demo/resume 로 재개")
                time.sleep(2.0)
                continue
            try:
                self._cycle()
            except Exception as exc:  # noqa: BLE001 - 사이클 하나가 죽어도 셀은 계속 돈다
                self.get_logger().error(f"사이클 예외 : {exc}")
                self._set(S.IDLE, f"예외 복구 : {exc}")
                time.sleep(2.0)

    def _cycle(self) -> None:
        started = time.time()

        # --- 벨트에 박스를 올린다. 누가 올리느냐가 두 시나리오의 차이다.
        if not self.station_box:
            if self.mode == "infeed":
                # 상류 라인이 올린다. 로봇은 관여하지 않는다.
                if not self._request_infeed():
                    time.sleep(1.0)
                    return
            else:
                # 순환 : 로봇이 원본 팔레트에서 꺼내 벨트에 되올린다.
                if not self._depalletize_to_belt():
                    time.sleep(1.0)
                    return

        # --- WAIT_BOX : 벨트가 정지 센서까지 실어 온다
        # infeed 모드에서는 _request_infeed가 이미 롤러를 돌려 두었다.
        self._set(S.WAIT_BOX, "벨트 이송 중")
        if self.mode != "infeed":
            self._belt(BeltCommand.Request.FEED_ONE)
        name = self._wait_station(float(self.get_parameter("belt_timeout").value))
        if not name:
            self._set(S.IDLE, "정지 센서에 박스가 오지 않았다")
            self._belt(BeltCommand.Request.STOP)
            return

        self._set(S.STOPPED, f"{name} 정지 센서 도달. 벨트 정지.")
        self._belt(BeltCommand.Request.STOP)

        # 롤러가 서고 박스가 멎기까지 기다린다. 실물 라인에서 스토퍼가 박스를
        # 받아 내고 흔들림이 잦아들기를 기다리는 것과 같다. 이 시간을 안 두면
        # 아직 미끄러지는 중인 박스를 찍어 판독 위치가 실제와 어긋난다.
        # 실측으로 정지 명령 뒤 박스가 440 -> 425 -> 433 mm로 흔들렸다.
        time.sleep(float(self.get_parameter("settle_time").value))

        # --- READ : 3회까지
        self._set(S.READ, "라벨 판독")
        retries = int(self.get_parameter("read_retries").value)
        label = None
        for attempt in range(1, retries + 1):
            label = self._read_label(attempt)
            if label:
                break
            time.sleep(0.4)

        if label is None:
            self._to_exception(name)
            return

        # --- QUERY : MES 조회. 이제 이 단계가 사이클의 분기점이다.
        #
        # 전에는 품목 이름을 화면에 띄우는 정도였고, 실패해도 그냥 쌓았다.
        # 규격이 여러 가지가 되면서 이 조회가 없으면 아무것도 못 한다.
        #   - 박스 높이를 모르면 컵을 어디까지 내릴지 모른다
        #   - 바닥 치수를 모르면 팔레트에 자리를 못 잡는다
        #   - 취급 구분을 모르면 쌓아도 되는 물건인지 모른다
        # 그래서 조회에 실패하면 예외 통으로 보낸다. 모르는 물건을 쌓는
        # 것보다 낫다. 실물 창고의 처리와 같다.
        self._set(S.QUERY, f"{label.code} 품목 조회")
        item = None
        if bool(self.get_parameter("use_mes").value) and self.mes_item.service_is_ready():
            req = ItemQuery.Request()
            req.code = label.code
            item = self.mes_item.call(req)

        if item is None or not item.found:
            self.get_logger().warn(f"{label.code}는 MES에 없는 코드다. 예외 통으로 보낸다.")
            self._to_exception(name, label.code, "미등록 코드")
            return

        box_size = list(item.box_size) if len(item.box_size) == 3 else list(
            self.cell.default_box_size)
        handling = item.handling or "normal"
        self._set(
            S.QUERY,
            f"{label.code} = {item.name} ({item.category}) "
            f"규격 {item.kind} {box_size[2]*1000:.0f}mm {item.weight_kg:.2f}kg [{handling}]",
        )
        self._event(
            "READ", label.code,
            f"conf={label.confidence:.2f},name={item.name},kind={item.kind},"
            f"handling={handling}",
        )

        # 취급 구분에 따른 분기. 라벨에도 빨간 띠로 찍혀 있다.
        #   hazmat   위험물. 일반 팔레트에 못 올린다.
        #   oversize 규격 초과. 팔레트에 안 들어간다.
        # 둘 다 사람이 따로 처리한다. 로봇은 예외 통까지만 옮긴다.
        if handling in ("hazmat", "oversize"):
            self.get_logger().warn(
                f"{label.code}는 {handling}이다. 팔레트에 올리지 않고 예외 통으로 보낸다."
            )
            self._to_exception(name, label.code, handling, box_size=box_size)
            return

        # --- PLAN : 다음 적재 자리. 이 박스 크기로 빈 자리를 찾는다.
        self._set(S.PLAN, f"팔레트 {self.target} 다음 자리")
        req = NextSlot.Request()
        req.pallet_id = self.target
        req.box_size = [float(v) for v in box_size]
        slot = self.next_slot.call(req)
        if slot is None or not slot.success:
            # 이 박스가 안 들어간다고 팔레트가 다 찬 것은 아니다.
            # 규격이 여러 가지가 되면서 "자리 없음"과 "가득"이 갈렸다.
            # 큰 상자가 못 들어가도 작은 상자는 들어갈 자리가 남아 있다.
            # 그래서 반대쪽 팔레트를 한 번 본 다음에 판단한다.
            others = [p for p in self.cell.pallet_ids if p != self.target]
            for alt in others:
                req.pallet_id = alt
                slot = self.next_slot.call(req)
                if slot is not None and slot.success:
                    self.get_logger().info(
                        f"팔레트 {self.target}에 {box_size[0]*1000:.0f}x"
                        f"{box_size[1]*1000:.0f} 자리가 없어 팔레트 {alt}로 보낸다"
                    )
                    self.target = alt
                    break
            else:
                self._pallet_full()
                return

        # --- EXECUTE
        # 파지 높이는 이 박스의 상면이다. 규격마다 다르므로 MES가 준 높이를 쓴다.
        pick = down_pose(
            label.center_x, label.center_y,
            self.cell.read_top_z_for(box_size[2]), label.yaw,
        )
        self._set(
            S.EXECUTE,
            f"{label.code} -> P{self.target} #{slot.index} (층 {slot.layer})",
        )
        if not self._pick_place(pick, slot.place_pose, name, box_size=box_size):
            self._event("EXCEPTION", label.code, "픽앤플레이스 실패")
            self._set(S.IDLE, "픽앤플레이스 실패. 다음 사이클로 넘어간다.")
            return

        # --- 집었는지 확인. 정지 센서가 아직 박스를 보고 있으면 못 집은 것이다.
        #
        # ES45는 잡았는지 되물을 수 없다(gripper_driver 주석 참고). 그래서
        # 동작을 끝냈다는 것과 옮겼다는 것은 다른 이야기다. 실제로 이걸 안
        # 보다가 크게 당했다. 흡착이 헛나가 박스가 벨트에 그대로 남았는데
        # 사이클은 성공으로 기록됐고, 다음 사이클이 같은 박스를 다시 읽어
        # 또 헛집기를 여섯 번 반복하며 팔레트를 유령으로 채웠다.
        #
        # 확인에 새 장비가 필요하지 않다. 정지 센서(광전 센서)는 이미 있고,
        # 집어 갔으면 그 자리가 비어야 한다. 실물 팔레타이저의 표준 인터록이다.
        # 기획서의 C2 적재 확인은 이것과 별개로, 쌓인 모양을 보는 일이다.
        if self._still_at_station(name):
            self._event("EXCEPTION", label.code, "헛집기")
            self._set(S.IDLE, f"{name}가 판독 자리에 그대로 있다. 헛집었다.")
            self.get_logger().error(
                f"헛집기. {name}를 집지 못했는데 동작은 끝났다. "
                "적재 기록을 남기지 않고 다시 시도한다."
            )
            return

        # --- RECORD
        #
        # 기록하기 전에 C2로 실제로 놓였는지 본다(기획서 D 항목).
        # 로봇이 "완료"라고 답한 것은 궤적을 끝냈다는 뜻일 뿐이다. 이송 중에
        # 떨어뜨렸거나 옆에 놓았어도 그렇게 답한다. 그것을 기록하면 팔레트
        # 장부와 실물이 어긋나고, 다음 박스는 없는 층 위에 놓인다.
        self._set(S.RECORD, "적재 확인")
        seen = self._verify_stack(slot, box_size)
        if seen is False:
            # 기록하지 않는다. next_slot의 예약은 commit이 와야 확정되므로
            # 그냥 두면 그 자리는 다음 박스에게 다시 나간다.
            self._event("PLACE_FAIL", label.code, f"p{self.target},{slot.layer},{slot.index}")
            self.get_logger().error(
                f"{label.code}를 놓았다는데 C2에 안 보인다. 기록하지 않는다."
            )
            self._set(S.IDLE, "적재 확인 실패. 사람이 봐야 한다.")
            self.running = False
            return

        self._set(S.RECORD, "적재 기록")
        self.commit_pub.publish(String(data=f"{self.target},{slot.index},{label.code}"))
        photo = self._photo()
        self._event("PLACE", label.code, f"p{self.target},{slot.layer},{slot.index},{photo}")

        self.cycles += 1
        self.last_cycle = time.time() - started
        self.get_logger().info(
            f"사이클 {self.cycles} 완료 {self.last_cycle:.1f} s (목표 20 s)"
        )

        target_state = self.pallets.get(self.target)
        if target_state is not None and target_state.full:
            self._pallet_full()
        else:
            self._set(S.IDLE, "다음 박스")

    def _request_infeed(self) -> bool:
        """상류 라인에 박스 하나를 요청한다. infeed 모드 전용.

        롤러를 **먼저** 돌린다. 순서가 중요하다. 멈춘 롤러 위에 떨어진 박스는
        안착과 동시에 잠들고, 그다음에 롤러가 돌아도 다시 깨어나지 않는다
        (위로 50 N을 걸어도 안 뜰 만큼 얼어 있다). 이미 도는 롤러 위에
        떨어뜨리면 그 문제가 생길 틈이 없다.
        """
        if not self.feed_next.service_is_ready():
            self.get_logger().warn("상류 라인(/feeder/next)이 없다", throttle_duration_sec=10.0)
            return False
        self._belt(BeltCommand.Request.FEED_ONE)
        res = self.feed_next.call(Trigger.Request())
        if res is None or not res.success:
            detail = res.message if res else "무응답"
            if "대기 중인 박스가 없다" in detail:
                self._to_circulation()
            else:
                self.get_logger().info(f"투입 대기 : {detail}", throttle_duration_sec=5.0)
            return False
        self._set(S.WAIT_BOX, f"{res.message} 컨베이어 진입")
        return True

    def _to_circulation(self) -> None:
        """상류 로트를 다 받았다. 이제부터는 순환으로 돈다.

        쌓아 둔 팔레트에서 로봇이 하나씩 꺼내 벨트로 되올리고, 판독을 거쳐
        반대쪽 팔레트에 쌓는다. 기획서의 "순환"이 여기서 시작된다.

        박스를 순간이동시키거나 새로 만들지 않는다. 로봇이 실제로 옮긴다.
        데모로도 그림이 낫고, 실물에서 지게차가 하는 일과도 대응된다.
        """
        if self.mode == "circulate":
            return
        self.mode = "circulate"
        # 지금 쌓고 있던 팔레트가 이제 꺼내 올 팔레트가 된다.
        self.source = self.target
        others = [p for p in self.cell.pallet_ids if p != self.source]
        self.target = others[0] if others else self.source
        self._set(
            S.IDLE,
            f"상류 로트를 다 받았다. 순환으로 전환 : "
            f"팔레트 {self.source}에서 꺼내 {self.target}에 쌓는다.",
        )
        self._event("SWAP", "", f"infeed->circulate source={self.source},target={self.target}")

    def _pallet_full(self) -> None:
        """대상 팔레트가 찼다.

        투입 모드에서 팔레트가 먼저 차는 경우(상류에 박스가 남았는데 자리가
        없는 경우)도 순환과 같이 처리한다. 꺼내는 쪽과 쌓는 쪽을 바꾸면
        로봇이 찬 팔레트에서 꺼내 반대쪽에 쌓기 시작한다.

        투입 모드에서는 _swap_pallets를 쓰면 안 된다. 그때 source는 0이다.
        상류 라인이 박스를 주므로 꺼내 올 팔레트가 없다는 뜻으로 둔 값인데,
        그걸 그대로 뒤집으면 source가 0이 되어 "팔레트 0에서 꺼내 1에 쌓는다"
        같은 말이 나온다(실측). 투입에서 순환으로 넘어가는 길은 하나뿐이다.
        """
        if self.mode == "infeed":
            self._to_circulation()
            return
        self._swap_pallets()

    def _depalletize_to_belt(self) -> bool:
        """원본 팔레트에서 하나 꺼내 벨트 투입 자리에 올린다. 기록의 역순."""
        self._set(S.DEPALLETIZE, f"팔레트 {self.source}에서 반출")
        req = ReleaseSlot.Request()
        req.pallet_id = self.source
        req.index = -1
        res = self.release.call(req)
        if res is None or not res.success:
            # 두 팔레트가 다 비었으면 꺼낼 것이 없다. 그대로 두면 1초마다
            # 서로를 번갈아 쳐다보며 영원히 돈다. 데모 화면에 그 로그만
            # 흐르므로 한 번 알리고 멈춘다.
            self._empty_swaps += 1
            self._swap_pallets()
            if self._empty_swaps >= len(self.cell.pallet_ids):
                self.running = False
                self._set(S.IDLE, "두 팔레트가 다 비었다. 순환을 멈춘다. "
                                  "다시 돌리려면 /demo/start.")
            return False
        self._empty_swaps = 0

        # 꺼낸 박스의 규격을 MES에 묻는다. 벨트에 내려놓을 높이가 규격마다
        # 다르기 때문이다. 코드는 반출 응답이 알려 준다.
        size = list(self.cell.default_box_size)
        if res.code and self.mes_item.service_is_ready():
            q = ItemQuery.Request()
            q.code = res.code
            it = self.mes_item.call(q)
            if it and it.found and len(it.box_size) == 3:
                size = list(it.box_size)
        place = down_pose(
            self.cell.infeed_x, self.cell.belt_center_y,
            self.cell.read_top_z_for(size[2]),
        )
        # 어느 물리 박스인지는 모른다(고르는 일은 gripper_driver가 한다).
        # 하지만 '무언가를 들고 있다'는 사실은 MoveIt에 반드시 알려야 한다.
        # 안 알리면 손에 든 박스를 모른 채 1층 위를 스치는 궤적이 나온다.
        carried = f"carried_p{self.source}_s{res.index}"
        if not self._pick_place(res.pick_pose, place, carried, home=False, box_size=size):
            # 자리는 이미 기록에서 비워졌는데 박스는 팔레트에 그대로 있다.
            # 되돌리지 않으면 다음 사이클이 빈 자리를 집으러 간다.
            self.commit_pub.publish(String(data=f"{self.source},{res.index},{res.code}"))
            self._set(S.IDLE, f"반출 이송 실패. P{self.source} #{res.index} 기록 복구.")
            return False
        self._event("DEPAL", res.code, f"p{self.source},{res.index}")
        return True

    def _to_exception(self, box_name: str) -> None:
        """판독 3회 실패. 예외 통으로 보낸다."""
        self.exceptions += 1
        self._set(S.EXCEPTION, f"판독 3회 실패. {box_name}를 예외 통으로.")
        self._event("EXCEPTION", "", "판독 실패")

        pick = down_pose(*self.cell.read_station_xy(), self.cell.read_top_z)
        x, y, z = self.cell.exception_drop_pose()
        # 통 위에서 놓는다. 통 안으로 내려놓을 필요는 없다.
        if not self._pick_place(pick, down_pose(x, y, z), box_name, box_size=size):
            self.jam_tries[box_name] = self.jam_tries.get(box_name, 0) + 1
            tries = self.jam_tries[box_name]
            limit = int(self.get_parameter("jam_retries").value)
            if tries < limit:
                self._set(S.IDLE, f"예외 통 이송 실패 ({tries}/{limit})")
                return
            self._jam(box_name)
            return
        self.jam_tries.pop(box_name, None)
        self._set(S.IDLE, "예외 처리 완료")

    def _jam(self, box_name: str) -> None:
        """치울 수 없는 박스. 라인을 세우고 사람을 부른다."""
        self.jammed = box_name
        self._belt(BeltCommand.Request.STOP)
        self._set(S.IDLE, f"잼 : {box_name}를 로봇이 치우지 못했다. 라인 정지.")
        self._event("JAM", "", box_name)
        self.get_logger().error(
            f"잼. {box_name}가 판독도 파지도 안 되는 자세로 정지 위치에 있다. "
            "사람이 치워야 한다. 치운 뒤 /demo/resume 로 다시 시작한다."
        )

    def _on_resume(self, _req, res):
        """사람이 잼을 치웠다고 알린다."""
        was = self.jammed
        self.jammed = ""
        self.jam_tries.clear()
        res.success = True
        res.message = f"{was or '없음'} 잼 해제" if was else "잼 상태가 아니었다"
        self.get_logger().info(res.message)
        return res

    def _swap_pallets(self) -> None:
        """대상 팔레트가 찼다. 꺼내는 쪽과 쌓는 쪽을 바꾼다."""
        self.source, self.target = self.target, self.source
        self._set(
            S.IDLE,
            f"팔레트 전환 : 이제 {self.source}에서 꺼내 {self.target}에 쌓는다",
        )
        self._event("SWAP", "", f"source={self.source},target={self.target}")


def main() -> None:
    rclpy.init()
    node = TaskManager()
    node.worker.start()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
