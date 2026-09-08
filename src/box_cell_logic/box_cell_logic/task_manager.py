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

        self.station_box = ""
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
        self.feed_next = self.create_client(Trigger, "/feeder/next", callback_group=self.cb)
        self.feed_recycle = self.create_client(Trigger, "/feeder/recycle", callback_group=self.cb)

        self.commit_pub = self.create_publisher(String, "/pallet/commit", 10)
        self.event_pub = self.create_publisher(String, "/mes/event", 10)
        self.state_pub = self.create_publisher(CellState, "/cell/state", 10)

        self.create_subscription(String, "/conveyor/box_at_station", self._on_station, 10)
        self.create_subscription(LabelDetection, "/perception/label", self._on_label, 10)
        self.create_subscription(PalletState, "/pallet/state", self._on_pallet, 10)
        self.create_subscription(String, "/cell/command", self._on_command, 10)

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

    def _read_label(self, attempt: int) -> LabelDetection | None:
        """판독 한 번. 3회째는 C4 손목 카메라로 근접 재시도한다(기획서 5.4)."""
        client = self.read_c1
        who = "C1"
        if attempt >= 3 and self.read_c4.service_is_ready():
            client = self.read_c4
            who = "C4 손목"
        self._label_event.clear()
        self.label = None
        res = client.call(Trigger.Request())
        if not (res and res.success):
            self.get_logger().warn(f"{who} 판독 실패 ({attempt}회) : {res.message if res else '무응답'}")
            return None
        if not self._label_event.wait(float(self.get_parameter("read_timeout").value)):
            self.get_logger().warn("pose_resolver 응답이 없다")
            return None
        return self.label if (self.label and self.label.ok) else None

    def _pick_place(self, pick: Pose, place: Pose, obj: str, home: bool = True) -> bool:
        goal = PickPlace.Goal()
        goal.pick_pose = pick
        goal.place_pose = place
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
        self._set(S.WAIT_BOX, "벨트 이송 중")
        self._belt(BeltCommand.Request.FEED_ONE)
        name = self._wait_station(float(self.get_parameter("belt_timeout").value))
        if not name:
            self._set(S.IDLE, "정지 센서에 박스가 오지 않았다")
            self._belt(BeltCommand.Request.STOP)
            return

        self._set(S.STOPPED, f"{name} 정지 센서 도달. 벨트 정지.")
        self._belt(BeltCommand.Request.STOP)

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

        # --- QUERY : MES 품목 조회. 실패해도 사이클은 계속 간다.
        self._set(S.QUERY, f"{label.code} 품목 조회")
        item_name = ""
        if bool(self.get_parameter("use_mes").value) and self.mes_item.service_is_ready():
            req = ItemQuery.Request()
            req.code = label.code
            item = self.mes_item.call(req)
            if item and item.found:
                item_name = item.name
                self._set(S.QUERY, f"{label.code} = {item.name} ({item.category})")
            else:
                self.get_logger().warn(f"{label.code}는 MES에 없는 코드다")
        self._event("READ", label.code, f"conf={label.confidence:.2f},name={item_name}")

        # --- PLAN : 다음 적재 자리
        self._set(S.PLAN, f"팔레트 {self.target} 다음 자리")
        req = NextSlot.Request()
        req.pallet_id = self.target
        slot = self.next_slot.call(req)
        if slot is None or not slot.success:
            self._pallet_full()
            return

        # --- EXECUTE
        pick = down_pose(label.center_x, label.center_y, self.cell.read_top_z, label.yaw)
        self._set(
            S.EXECUTE,
            f"{label.code} -> P{self.target} #{slot.index} (층 {slot.layer})",
        )
        if not self._pick_place(pick, slot.place_pose, name):
            self._event("EXCEPTION", label.code, "픽앤플레이스 실패")
            self._set(S.IDLE, "픽앤플레이스 실패. 다음 사이클로 넘어간다.")
            return

        # --- RECORD
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
        """상류 라인에 박스 하나를 요청한다. infeed 모드 전용."""
        if not self.feed_next.service_is_ready():
            self.get_logger().warn("상류 라인(/feeder/next)이 없다", throttle_duration_sec=10.0)
            return False
        res = self.feed_next.call(Trigger.Request())
        if res is None or not res.success:
            detail = res.message if res else "무응답"
            if "대기 중인 박스가 없다" in detail:
                # 로트를 다 썼다. 팔레트가 아직 덜 찼다면 반출하고 다시 받는다.
                self._set(S.IDLE, "투입할 박스가 없다. 팔레트를 비우고 다시 받는다.")
                self._pallet_full()
            else:
                self.get_logger().info(f"투입 대기 : {detail}")
            return False
        self._set(S.WAIT_BOX, f"{res.message} 컨베이어 진입")
        return True

    def _pallet_full(self) -> None:
        """대상 팔레트가 찼다. 시나리오에 따라 처리가 다르다."""
        if self.mode == "infeed":
            self._unload_pallet()
        else:
            self._swap_pallets()

    def _unload_pallet(self) -> None:
        """팔레트 반출. 지게차가 가져가고 빈 팔레트가 들어온 것으로 본다."""
        self._set(S.DEPALLETIZE, f"팔레트 {self.target} 가득. 반출한다.")
        self._event("UNLOAD", "", f"p{self.target}")
        time.sleep(float(self.cell.scenario.get("unload_pause", 3.0)))

        if self.feed_recycle.service_is_ready():
            self.feed_recycle.call(Trigger.Request())
        # 적재 기록을 비운다. 슬롯을 하나씩 놓아 주면 scene_publisher가
        # 충돌체도 따라서 지운다.
        state = self.pallets.get(self.target)
        if state is not None:
            for index, occupied in enumerate(state.occupied):
                if occupied:
                    req = ReleaseSlot.Request()
                    req.pallet_id = self.target
                    req.index = index
                    self.release.call(req)
        self._set(S.IDLE, f"팔레트 {self.target} 비웠다. 다음 로트를 받는다.")

    def _depalletize_to_belt(self) -> bool:
        """원본 팔레트에서 하나 꺼내 벨트 투입 자리에 올린다. 기록의 역순."""
        self._set(S.DEPALLETIZE, f"팔레트 {self.source}에서 반출")
        req = ReleaseSlot.Request()
        req.pallet_id = self.source
        req.index = -1
        res = self.release.call(req)
        if res is None or not res.success:
            self._swap_pallets()
            return False

        place = down_pose(self.cell.infeed_x, self.cell.belt_center_y, self.cell.read_top_z)
        # 어느 물리 박스인지는 모른다(고르는 일은 gripper_driver가 한다).
        # 하지만 '무언가를 들고 있다'는 사실은 MoveIt에 반드시 알려야 한다.
        # 안 알리면 손에 든 박스를 모른 채 1층 위를 스치는 궤적이 나온다.
        carried = f"carried_p{self.source}_s{res.index}"
        if not self._pick_place(res.pick_pose, place, carried, home=False):
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
        if not self._pick_place(pick, down_pose(x, y, z), box_name):
            self._set(S.IDLE, "예외 통 이송 실패")
            return
        self._set(S.IDLE, "예외 처리 완료")

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
