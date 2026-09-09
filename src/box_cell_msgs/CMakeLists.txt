cmake_minimum_required(VERSION 3.16)
project(box_cell_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LabelDetection.msg"
  "msg/LabelCorners.msg"
  "msg/BoxPose.msg"
  "msg/BoxPoseArray.msg"
  "msg/CellState.msg"
  "msg/PalletState.msg"
  "srv/ItemQuery.srv"
  "srv/NextSlot.srv"
  "srv/VerifyStack.srv"
  "srv/ReleaseSlot.srv"
  "srv/Vacuum.srv"
  "srv/BeltCommand.srv"
  "action/PickPlace.action"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
