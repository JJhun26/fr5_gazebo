cmake_minimum_required(VERSION 3.16)
project(box_cell_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes config launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
