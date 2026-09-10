"""실물 셀. Gazebo 없이, 실제 FR5와 주변 장치로 돈다.

    ros2 launch box_cell_bringup real.launch.py robot_ip:=192.168.58.2

demo.launch.py의 hardware:=real과 뜨는 것은 거의 같다. 이 파일을 따로 두는
이유는 실물에만 있는 사정을 한곳에 적어 두기 위해서다. Gazebo가 없다는 것은
`/sim/boxes`(정답지)도 없고 `box_feeder`(상류 라인 역할)도 없다는 뜻이다.
그 자리에 무엇이 들어와야 하는지가 아래 인자와 주석이다.

먼저 읽을 것 : docs/real_robot_bringup.md, docs/digital_twin.md

이 파일이 띄우지 않는 것 — 밖에서 와야 한다
    카메라 드라이버   realsense2_camera. 토픽 이름을 시뮬과 맞춰 remap한다
                      (/c1_conveyor/color/image/compressed 등). cameras:=false로
                      두고 따로 띄운 뒤 이름만 맞추면 인식 노드는 그대로 돈다.
    IO 게이트웨이     /io/tool_do(진공 밸브), /io/conveyor_run(인버터),
                      /io/photo_eye(정지 센서). 배선과 함께 오는 노드다.
                      그 전에도 토픽은 나가고 들어오므로 타이밍은 확인된다.
    안전              비상정지, 라이트커튼, 속도/힘 제한. ROS를 거치지 않는다.
                      컨트롤러의 안전 입력에 직결한다. 이건 협상 대상이 아니다.

한 번에 다 붙이지 않는다. docs/real_robot_bringup.md 7절의 순서대로,
로봇만 → 그리퍼/컨베이어 → 카메라 순으로 하나씩 켠다. 그래서 인자로 끄고
켤 수 있게 두었다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("box_cell_bringup")
    moveit_share = get_package_share_directory("box_cell_moveit_config")

    robot_ip = LaunchConfiguration("robot_ip")
    perception_on = IfCondition(LaunchConfiguration("perception"))
    # 실물에는 시뮬 시각이 없다. 전부 벽시계로 돈다.
    real_time = {"use_sim_time": False}

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "control.launch.py")),
        launch_arguments={"hardware": "real", "robot_ip": robot_ip}.items(),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_share, "launch", "move_group.launch.py")),
        launch_arguments={"hardware": "real", "robot_ip": robot_ip}.items(),
    )

    motion = [
        Node(
            package="box_cell_motion",
            executable="motion_server",
            name="motion_server",
            output="screen",
            parameters=[real_time],
        ),
        # 실물 밸브. /gripper/vacuum 서비스는 시뮬과 같고, 밑에서 Bool 하나가
        # /io/tool_do로 나간다. ES45에는 피드백이 없으므로 응답은 여전히
        # "명령을 냈다"는 뜻이다. 잡았는지는 정지 센서와 C2 확인으로만 안다.
        Node(
            package="box_cell_motion",
            executable="gripper_driver",
            name="gripper_driver",
            output="screen",
            parameters=[
                real_time,
                {"mode": "real", "real_backend": LaunchConfiguration("io_backend")},
            ],
        ),
        # 계획 씬. 적재된 박스는 pallet_manager의 기록에서 온다.
        # 시뮬 정답지를 보지 않으므로 실물에서 그대로 돈다.
        Node(
            package="box_cell_motion",
            executable="scene_publisher",
            name="scene_publisher",
            output="screen",
            parameters=[real_time],
        ),
    ]

    # 인식. 영상 토픽 이름만 맞으면 시뮬과 같은 노드가 그대로 돈다.
    # 핸드아이 캘리브레이션 결과는 URDF의 카메라 조인트로 들어간다.
    # pose_resolver가 호모그래피를 TF에서 계산하므로 그것만으로 따라온다.
    perception = [
        Node(
            package="box_cell_perception",
            executable="label_reader",
            name="label_reader_c1",
            output="screen",
            parameters=[
                real_time,
                {"camera": "c1_conveyor", "image_topic": "/c1_conveyor/color/image/compressed"},
            ],
            condition=perception_on,
        ),
        Node(
            package="box_cell_perception",
            executable="label_reader",
            name="label_reader_c4",
            output="screen",
            parameters=[
                real_time,
                {"camera": "c4_wrist", "image_topic": "/c4_wrist/color/image/compressed"},
            ],
            condition=perception_on,
        ),
        Node(
            package="box_cell_perception",
            executable="pose_resolver",
            name="pose_resolver_c1",
            output="screen",
            parameters=[real_time, {"camera": "c1_conveyor"}],
            condition=perception_on,
        ),
        Node(
            package="box_cell_perception",
            executable="pose_resolver",
            name="pose_resolver_c4",
            output="screen",
            parameters=[real_time, {"camera": "c4_wrist"}],
            condition=perception_on,
        ),
        Node(
            package="box_cell_perception",
            executable="stack_check",
            name="stack_check",
            output="screen",
            parameters=[real_time, {"camera": "c2_pallet"}],
            condition=perception_on,
        ),
        Node(
            package="box_cell_perception",
            executable="camera_node",
            name="camera_c2",
            output="screen",
            parameters=[real_time, {"camera": "c2_pallet"}],
            condition=perception_on,
        ),
        Node(
            package="box_cell_perception",
            executable="camera_node",
            name="camera_c3",
            output="screen",
            parameters=[real_time, {"camera": "c3_scene"}],
            condition=perception_on,
        ),
    ]

    logic = [
        Node(
            package="box_cell_logic",
            executable="pallet_manager",
            name="pallet_manager",
            output="screen",
            # 실물에서는 기록이 곧 진실이다. 재부팅을 넘겨야 하므로
            # BOX_CELL_DATA_DIR을 /var/lib/box_cell 같은 곳으로 두고,
            # reset은 기본 false다. 지난 운전의 적재 상태에서 이어 간다.
            parameters=[real_time, {"reset": LaunchConfiguration("reset")}],
        ),
        Node(
            package="box_cell_logic",
            executable="mes_client",
            name="mes_client",
            output="screen",
            parameters=[real_time],
        ),
        # 인버터 기동/정지와 광전 센서. /belt/command는 시뮬과 같다.
        Node(
            package="box_cell_conveyor",
            executable="conveyor_driver",
            name="conveyor_driver",
            output="screen",
            parameters=[real_time, {"mode": "real"}],
        ),
        # 트윈의 출구. 실물 셀의 상태를 시뮬레이터와 같은 스키마로 낸다.
        # 이것이 있어야 시뮬과 실물을 같은 화면에서 견줄 수 있다.
        Node(
            package="box_cell_logic",
            executable="twin_bridge",
            name="twin_bridge",
            output="screen",
            parameters=[real_time, {"source": "real"}],
        ),
        # Dry Run 채점기. 실물에서도 그대로 돌린다. 시뮬 점수와 벌어지는
        # 항목이 곧 시뮬이 틀린 곳이다(docs/real_robot_bringup.md 7절).
        Node(
            package="box_cell_logic",
            executable="dry_run_scorer",
            name="dry_run_scorer",
            output="screen",
            parameters=[real_time],
        ),
    ]

    # 상태 기계는 나머지가 다 뜬 뒤에. 실물은 드라이버 연결이 더 오래 걸린다.
    # 그리고 기본이 autostart:=false다. 실물이 사람 없이 저 혼자 시작하면
    # 안 된다. 시작은 사람이 /cell/command로 준다.
    task_manager = TimerAction(
        period=12.0,
        actions=[
            Node(
                package="box_cell_logic",
                executable="task_manager",
                name="task_manager",
                output="screen",
                parameters=[real_time, {"auto_start": LaunchConfiguration("autostart")}],
            )
        ],
    )

    mes = ExecuteProcess(
        cmd=["ros2", "run", "box_cell_mes", "mes_server", "--port", "8020"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("mes")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", default_value="192.168.58.2"),
            # 실물은 사람이 시작을 잡는다. 기본값이 시뮬과 반대인 유일한 인자다.
            DeclareLaunchArgument("autostart", default_value="false"),
            DeclareLaunchArgument("perception", default_value="true"),
            DeclareLaunchArgument("mes", default_value="true"),
            DeclareLaunchArgument("reset", default_value="false"),
            # topic이면 /io/*로 신호를 낸다. none이면 아무것도 내지 않는다.
            # 배선 전 첫 시운전은 none으로 시작해 팔만 움직여 보는 것이 맞다.
            DeclareLaunchArgument("io_backend", default_value="topic"),
            control,
            move_group,
            *motion,
            *perception,
            *logic,
            task_manager,
            mes,
        ]
    )
