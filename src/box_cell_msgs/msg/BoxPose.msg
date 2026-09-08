# 시뮬레이터가 아는 박스 하나의 실제 상태. 정답지다.
# 실물에는 없는 정보이므로 로직 노드가 이것을 읽으면 안 된다.
# conveyor_driver, gripper_driver, Dry Run 채점만 쓴다.

string                  name        # box_1 .. box_8
geometry_msgs/Pose      pose        # world 기준
geometry_msgs/Twist     twist       # world 기준. 위치 차분으로 낸 값이다.
string                  code        # 이 박스에 붙은 라벨 문자열
int8                    pallet      # 0 = 컨베이어, 1/2 = 팔레트, -1 = 예외 통
int8                    slot        # 팔레트 위 슬롯 인덱스. 없으면 -1.
bool                    held        # 지금 석션에 붙어 있는가
bool                    on_belt     # 벨트 유효 구간 위에 있는가
