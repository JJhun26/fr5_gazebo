"""Gazebo Harmonic 쪽 전부 : 월드, 두 모델, 컨트롤러, 브리지, 박스.

띄우는 순서가 중요하다.
  1. gz sim (월드)
  2. robot_state_publisher 두 개 (로봇 / 셀 구조물)
  3. 두 모델 스폰. 로봇 안의 gz_ros2_control이 여기서 controller_manager를 연다.
  4. 컨트롤러 스포너. controller_manager가 열린 뒤라야 붙는다.
  5. 브리지와 box_spawner

3번과 4번 사이에 반드시 간격이 필요하다. gz_ros2_control은 모델이 월드에
들어간 다음에야 controller_manager를 만들기 때문에, 스포너를 먼저 띄우면
서비스를 못 찾고 죽는다. 여기서는 스폰 완료 이벤트에 걸어 둔다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

WORLD = "box_cell"


def generate_launch_description() -> LaunchDescription:
    desc_share = get_package_share_directory("box_cell_description")
    sim_share = get_package_share_directory("box_cell_sim")
    bringup_share = get_package_share_directory("box_cell_bringup")

    # headless=true면 gz 서버만 띄운다. 화면 없는 장비, CI, 원격 점검용이다.
    # 카메라 센서는 여전히 돌아가므로 판독 파이프라인까지 확인할 수 있다.
    headless = LaunchConfiguration("headless")

    robot_xacro = os.path.join(desc_share, "urdf", "box_cell_robot.urdf.xacro")
    cell_xacro = os.path.join(desc_share, "urdf", "cell_furniture.urdf.xacro")
    world_xacro = os.path.join(sim_share, "worlds", "box_cell.sdf.xacro")
    controllers = os.path.join(bringup_share, "config", "ros2_controllers.yaml")

    # ParameterValue로 감싸야 한다. Command가 내는 것은 문자열인데
    # launch_ros는 기본적으로 YAML로 읽으려 들고, URDF는 콜론과 대괄호가
    # 가득해서 반드시 파싱에 실패한다.
    robot_description = ParameterValue(
        Command(["xacro ", robot_xacro, " hardware:=gazebo", " controllers_yaml:=", controllers]),
        value_type=str,
    )
    cell_description = ParameterValue(Command(["xacro ", cell_xacro]), value_type=str)

    # 월드는 xacro라 먼저 펼쳐 둔다. gz는 xacro를 모른다.
    world_sdf = os.path.join(os.environ.get("HOME", "/tmp"), ".box_cell", "box_cell.sdf")
    expand_world = ExecuteProcess(
        cmd=["bash", "-c", f"mkdir -p $(dirname {world_sdf}) && xacro {world_xacro} -o {world_sdf}"],
        output="screen",
    )

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            # -r : 바로 돌린다. 멈춘 채로 뜨면 컨트롤러가 붙지 못한다.
            # -s : 서버만. headless일 때 붙는다.
            "gz_args": [
                world_sdf,
                " -r -v 2 ",
                PythonExpression(["'-s' if '", headless, "' == 'true' else ''"]),
            ],
            "on_exit_shutdown": "true",
        }.items(),
    )

    rsp_robot = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # 셀 구조물은 별도 네임스페이스에 둔다. 로봇과 같은 /robot_description을
    # 쓰면 MoveIt이 상판과 갠트리를 로봇 링크로 착각한다.
    rsp_cell = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="cell_state_publisher",
        namespace="cell",
        output="screen",
        parameters=[{"robot_description": cell_description, "use_sim_time": True}],
        remappings=[("/tf", "/tf"), ("/tf_static", "/tf_static")],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot",
        output="screen",
        arguments=["-topic", "/robot_description", "-name", "fr5", "-allow_renaming", "false"],
    )
    spawn_cell = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_cell",
        output="screen",
        arguments=["-topic", "/cell/robot_description", "-name", "cell", "-allow_renaming", "false"],
    )

    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    jtc = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[
            {
                "config_file": os.path.join(sim_share, "config", "bridge.yaml"),
                "use_sim_time": True,
            }
        ],
    )

    # 영상은 전용 브리지로. 파라미터 브리지보다 복사가 한 번 적다.
    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="image_bridge",
        output="screen",
        arguments=[
            "/c1_conveyor/color/image",
            "/c2_pallet/color/image",
            "/c3_scene/color/image",
            "/c4_wrist/color/image",
        ],
        remappings=[
            ("/c1_conveyor/color/image", "/c1_conveyor/color/image_raw"),
            ("/c2_pallet/color/image", "/c2_pallet/color/image_raw"),
            ("/c3_scene/color/image", "/c3_scene/color/image_raw"),
            ("/c4_wrist/color/image", "/c4_wrist/color/image_raw"),
        ],
        parameters=[{"use_sim_time": True}],
    )

    # 상류 라인 역할. 박스를 만들고 벨트로 들여보낸다.
    # 시나리오(infeed / circulate)는 cell.yaml이 정한다.
    box_feeder = Node(
        package="box_cell_sim",
        executable="box_feeder",
        name="box_feeder",
        output="screen",
        parameters=[{"world": WORLD, "use_sim_time": True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false"),
            # 메시와 라벨 텍스처를 gz가 찾을 수 있게 한다.
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                os.pathsep.join(
                    [
                        os.path.dirname(desc_share),
                        os.path.dirname(sim_share),
                        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
                    ]
                ),
            ),
            expand_world,
            RegisterEventHandler(OnProcessExit(target_action=expand_world, on_exit=[gz])),
            rsp_robot,
            rsp_cell,
            spawn_cell,
            spawn_robot,
            # 로봇이 월드에 들어간 뒤라야 controller_manager가 열린다.
            RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[jsb])),
            RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[jtc])),
            bridge,
            image_bridge,
            box_feeder,
        ]
    )
