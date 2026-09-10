"""mock 하드웨어의 제어 계층. control.launch.py를 hardware:=mock으로 부른다.

내용은 control.launch.py로 옮겼다. 실물(real)도 같은 구조가 필요한데
(Gazebo 밖이므로 ros2_control_node를 직접 띄워야 한다) 두 벌로 두면
한쪽만 고치는 날이 온다. 이 파일은 예전 이름으로 부르던 곳을 위해 남긴다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    control = os.path.join(
        get_package_share_directory("box_cell_bringup"), "launch", "control.launch.py"
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(control),
                launch_arguments={"hardware": "mock"}.items(),
            )
        ]
    )
