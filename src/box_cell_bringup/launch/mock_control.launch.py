"""mock 하드웨어의 제어 계층.

Gazebo 모드에서는 gz_ros2_control 플러그인이 로봇 모델 안에서
controller_manager를 열어 준다. mock 모드에는 그런 것이 없으므로
`ros2_control_node`를 직접 띄워야 한다. 기획서 R2("mock으로 실물 없이
MoveIt2까지 전부 구동")가 성립하려면 이 파일이 필요하다.

여기서 뜨는 것은 Gazebo 모드와 같은 이름의 컨트롤러 두 개다. 그래야
motion_server와 MoveIt이 두 모드에서 똑같이 동작한다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    desc_share = get_package_share_directory("box_cell_description")
    bringup_share = get_package_share_directory("box_cell_bringup")

    robot_xacro = os.path.join(desc_share, "urdf", "box_cell_robot.urdf.xacro")
    cell_xacro = os.path.join(desc_share, "urdf", "cell_furniture.urdf.xacro")
    controllers = os.path.join(bringup_share, "config", "ros2_controllers.yaml")

    # ParameterValue로 감싸야 한다. Command가 내는 것은 문자열인데
    # launch_ros는 기본적으로 YAML로 읽으려 들고, URDF는 콜론과 대괄호가
    # 가득해서 반드시 파싱에 실패한다.
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", robot_xacro, " hardware:=mock"]), value_type=str
        )
    }
    cell_description = {
        "robot_description": ParameterValue(Command(["xacro ", cell_xacro]), value_type=str)
    }

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # 셀 구조물의 TF. 갠트리 카메라 위치가 여기서 나온다.
    rsp_cell = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="cell_state_publisher",
        namespace="cell",
        output="screen",
        parameters=[cell_description],
    )

    control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[robot_description, controllers],
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

    return LaunchDescription(
        [
            rsp,
            rsp_cell,
            control,
            jsb,
            RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[jtc])),
        ]
    )
