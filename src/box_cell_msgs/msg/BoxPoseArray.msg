# 시뮬레이터가 아는 박스 전체의 실제 상태. 정답지다.
#
# 실물에는 이런 토픽이 없다. 로직 노드(task_manager, pallet_manager,
# label_reader)가 이걸 읽으면 시뮬레이터에서만 되는 데모가 된다.
# 읽어도 되는 곳은 세 군데뿐이다.
#   conveyor_driver : 벨트 위 박스에 힘을 주려면 어느 놈인지 알아야 한다
#   gripper_driver  : 석션이 붙일 대상을 고른다 (실물에서는 진공이 물리로 한다)
#   dry_run 채점    : 맞게 쌓였는지 사람 대신 확인한다

std_msgs/Header header
BoxPose[] boxes
