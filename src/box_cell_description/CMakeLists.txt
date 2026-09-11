cmake_minimum_required(VERSION 3.16)
project(box_cell_description)

find_package(ament_cmake REQUIRED)

# launch는 여기 없다. 이 패키지는 형상과 제원만 담는다(런치는 box_cell_bringup).
# 없는 디렉터리를 적어 두면 ament_cmake_symlink_install이 빌드를 세운다.
# 평범한 install(DIRECTORY)는 조용히 넘어가지만 colcon은 --symlink-install로
# 돌므로 새로 클론한 워크스페이스가 첫 빌드에서 바로 걸린다.
install(
  DIRECTORY urdf meshes config rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
