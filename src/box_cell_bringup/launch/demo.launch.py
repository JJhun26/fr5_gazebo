"""물류 로봇 데모 전체. 이 하나로 셀이 뜬다.

    ros2 launch box_cell_bringup demo.launch.py

계층은 기획서 4절 "ROS2 노드 구성" 그대로다.
    하드웨어  gz_ros2_control + joint_trajectory_controller
    동작      move_group, motion_server, gripper_driver
    인식      camera_node x2, label_reader x2, pose_resolver
    논리      task_manager, pallet_manager, mes_client, conveyor_driver

인자
    hardware:=gazebo|mock   기본 gazebo. mock은 Gazebo 없이 MoveIt까지만.
    autostart:=true|false   false면 상태 기계가 IDLE에서 기다린다.
                            데모 시작 순간을 사람이 잡고 싶을 때 쓴다.
    rviz:=true|false        RViz 동시 실행
    mes:=true|false         MES 서버 동시 실행
    reset:=true|false       기본 true. 이전 실행의 적재 기록을 버리고
                            팔레트 1이 가득 찬 초기 배치로 시작한다.
    headless:=true|false    Gazebo GUI 없이 서버만. 카메라 센서는 그대로 돈다.
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
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    sim_share = get_package_share_directory("box_cell_sim")
    moveit_share = get_package_share_directory("box_cell_moveit_config")

    hardware = LaunchConfiguration("hardware")
    autostart = LaunchConfiguration("autostart")
    use_sim = PythonExpression(["'", hardware, "' == 'gazebo'"])
    sim_time = {"use_sim_time": use_sim}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_share, "launch", "gazebo.launch.py")),
        condition=IfCondition(use_sim),
        launch_arguments={"headless": LaunchConfiguration("headless")}.items(),
    )

    # mock 모드에는 gz_ros2_control이 없으므로 controller_manager를 직접 띄운다.
    mock_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("box_cell_bringup"), "launch", "mock_control.launch.py"
            )
        ),
        condition=UnlessCondition(use_sim),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_share, "launch", "move_group.launch.py")),
        launch_arguments={"hardware": hardware}.items(),
    )

    motion = [
        Node(
            package="box_cell_motion",
            executable="motion_server",
            name="motion_server",
            output="screen",
            parameters=[sim_time],
        ),
        Node(
            package="box_cell_motion",
            executable="gripper_driver",
            name="gripper_driver",
            output="screen",
            parameters=[sim_time, {"mode": "sim"}],
        ),
        Node(
            package="box_cell_motion",
            executable="scene_publisher",
            name="scene_publisher",
            output="screen",
            parameters=[sim_time],
        ),
    ]

    # 인식. 판독은 C1 한 대로 끝나고, C4는 실패 시 근접 폴백이다(기획서 5.4).
    perception = [
        Node(
            package="box_cell_perception",
            executable="label_reader",
            name="label_reader_c1",
            output="screen",
            parameters=[
                sim_time,
                {"camera": "c1_conveyor", "image_topic": "/c1_conveyor/color/image/compressed"},
            ],
        ),
        Node(
            package="box_cell_perception",
            executable="label_reader",
            name="label_reader_c4",
            output="screen",
            parameters=[
                sim_time,
                {"camera": "c4_wrist", "image_topic": "/c4_wrist/color/image/compressed"},
            ],
        ),
        # 적재 결과 확인. C2 팔레트 탑뷰의 깊이로 본다(기획서 D 항목).
        # task_manager가 RECORD 직전에 부른다. 없으면 확인 없이 진행한다.
        Node(
            package="box_cell_perception",
            executable="stack_check",
            name="stack_check",
            output="screen",
            parameters=[sim_time, {"camera": "c2_pallet"}],
        ),
        # 디지털 트윈 브리지(기획서 D4). 셀 상태를 JSON 한 창구로 낸다.
        # 파일 /tmp/box_cell/twin.json, 토픽 /twin/state, HTTP :8030/twin.
        # 내기만 하고 받지 않는다. 죽어도 셀은 계속 돈다.
        Node(
            package="box_cell_logic",
            executable="twin_bridge",
            name="twin_bridge",
            output="screen",
            parameters=[sim_time, {"source": "sim"}],
        ),
        # Dry Run 채점기(기획서 D3). 운전을 지켜보며 처리량, 사이클 시간,
        # 예외 사유, 적재 정확도를 모아 /tmp/box_cell/dry_run.json에 쓴다.
        # 동작에 관여하지 않는다. 보기만 한다.
        Node(
            package="box_cell_logic",
            executable="dry_run_scorer",
            name="dry_run_scorer",
            output="screen",
            parameters=[sim_time],
        ),
        Node(
            package="box_cell_perception",
            executable="pose_resolver",
            name="pose_resolver_c1",
            output="screen",
            parameters=[sim_time, {"camera": "c1_conveyor"}],
        ),
        # C4에도 변환기가 있어야 한다. 없으면 손목 카메라가 QR을 읽어도
        # 그 결과가 LabelDetection이 되지 못하고 버려진다. 로그에는
        # "pose_resolver 응답이 없다"로만 나와서 원인을 찾기 어렵다.
        # 판독 평면은 C1과 같다. 둘 다 벨트 위 박스 상면을 본다.
        Node(
            package="box_cell_perception",
            executable="pose_resolver",
            name="pose_resolver_c4",
            output="screen",
            parameters=[sim_time, {"camera": "c4_wrist"}],
        ),
        Node(
            package="box_cell_perception",
            executable="camera_node",
            name="camera_c3",
            output="screen",
            parameters=[sim_time, {"camera": "c3_scene"}],
        ),
        Node(
            package="box_cell_perception",
            executable="camera_node",
            name="camera_c2",
            output="screen",
            parameters=[sim_time, {"camera": "c2_pallet"}],
        ),
    ]

    logic = [
        Node(
            package="box_cell_logic",
            executable="pallet_manager",
            name="pallet_manager",
            output="screen",
            # initial_full은 넘기지 않는다. cell.yaml의 시나리오가 정한다.
            parameters=[sim_time, {"reset": LaunchConfiguration("reset")}],
        ),
        Node(
            package="box_cell_logic",
            executable="mes_client",
            name="mes_client",
            output="screen",
            parameters=[sim_time],
        ),
        Node(
            package="box_cell_conveyor",
            executable="conveyor_driver",
            name="conveyor_driver",
            output="screen",
            parameters=[sim_time],
        ),
    ]

    # 상태 기계는 나머지가 다 뜬 뒤에 붙인다. 먼저 뜨면 연결을 기다리며
    # 로그만 채우는데, 데모 준비 중에 그 로그가 진짜 문제를 가린다.
    task_manager = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="box_cell_logic",
                executable="task_manager",
                name="task_manager",
                output="screen",
                parameters=[sim_time, {"auto_start": autostart}],
            )
        ],
    )

    # ros2 run으로 띄운다. launch_ros의 Node는 --ros-args를 붙이는데
    # 이 서버는 argparse라 그걸 못 읽는다.
    mes = ExecuteProcess(
        cmd=["ros2", "run", "box_cell_mes", "mes_server", "--port", "8020"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("mes")),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            os.path.join(get_package_share_directory("box_cell_description"), "rviz", "box_cell.rviz"),
        ],
        parameters=[sim_time],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware", default_value="gazebo"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("mes", default_value="true"),
            # 데모는 매번 같은 상태에서 출발해야 한다. 이전 실행의 적재 기록이
            # 남아 있으면 빈 팔레트에서 시작해 아무 일도 일어나지 않는다.
            DeclareLaunchArgument("reset", default_value="true"),
            # 화면 없이 서버만 띄운다. 카메라는 그대로 돈다.
            DeclareLaunchArgument("headless", default_value="false"),
            gazebo,
            mock_control,
            move_group,
            *motion,
            *perception,
            *logic,
            task_manager,
            mes,
            rviz,
        ]
    )
