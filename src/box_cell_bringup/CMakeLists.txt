cmake_minimum_required(VERSION 3.16)
project(box_cell_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
