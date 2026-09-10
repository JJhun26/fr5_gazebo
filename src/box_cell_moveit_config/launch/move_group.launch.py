"""MoveIt2 move_group.

hardware 인자가 URDF의 ros2_control 플러그인을 고른다.
  gazebo  gz_ros2_control
  mock    mock_components/GenericSystem  (기획서 R2 : 실물 없이 전부 구동)
  real    FAIRINO 드라이버

플래너는 셋을 함께 올린다. 기획서 5.1대로 평시에는 Pilz(PTP/LIN)로 잇고
충돌로 실패할 때만 OMPL이 나선다. 어느 쪽이 궤적을 냈는지는 motion_server
로그에 남는다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def setup(context, *_args, **_kwargs):
    hardware = LaunchConfiguration("hardware").perform(context)
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    use_sim_time = hardware == "gazebo"

    moveit = (
        MoveItConfigsBuilder("box_cell", package_name="box_cell_moveit_config")
        .robot_description(mappings={"hardware": hardware, "robot_ip": robot_ip})
        .robot_description_semantic(file_path="config/box_cell.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="pilz_industrial_motion_planner",
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
        )
        .to_moveit_configs()
    )

    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit.to_dict(),
                {
                    "use_sim_time": use_sim_time,
                    # 계획 장면 갱신이 늦으면 방금 놓은 박스를 모르는 채로
                    # 다음 궤적을 낸다. 2층 적재에서 바로 사고가 난다.
                    "publish_planning_scene_hz": 20.0,
                    # 궤적 실행 감시. 컨트롤러가 늦으면 그냥 기다리는 편이
                    # 데모에서는 낫다. 중단하면 팔이 박스를 든 채 멈춘다.
                    "trajectory_execution.allowed_execution_duration_scaling": 2.5,
                    "trajectory_execution.allowed_goal_duration_margin": 1.0,
                    "trajectory_execution.allowed_start_tolerance": 0.02,
                },
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware", default_value="gazebo"),
            # hardware:=real일 때 URDF의 fairino_hardware 플러그인에 넘어간다.
            # move_group도 같은 robot_description을 써야 실물과 계획이 어긋나지 않는다.
            DeclareLaunchArgument("robot_ip", default_value="192.168.58.2"),
            OpaqueFunction(function=setup),
        ]
    )
