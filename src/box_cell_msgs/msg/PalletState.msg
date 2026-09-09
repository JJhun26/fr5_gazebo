# 팔레트 한 장에 실제로 무엇이 어디에 놓였는가. pallet_manager가 원본이다.
#
# 전에는 길이 8의 bool 배열이었다. 규격이 하나였을 때는 "슬롯 3번이 찼다"가
# 곧 위치였기 때문이다. 규격이 여러 가지가 되면서 고정 격자가 사라졌고,
# 자리는 그때그때 빈 곳을 찾아 정해진다. 그래서 무엇이 어디에 얼마만 한
# 크기로 놓였는지를 그대로 실어 나른다.
#
# 배열 인덱스가 곧 적재 순서다. 반출은 역순이어야 한다. 2층을 먼저 꺼내지
# 않으면 무너진다.

uint8   pallet_id
uint8   count
bool    full            # 더 놓을 자리가 없다

string[]                codes   # 라벨 문자열
geometry_msgs/Pose[]    poses   # 박스 **중심**, world 기준
geometry_msgs/Vector3[] sizes   # 놓인 자세에서의 치수 (yaw 반영)
uint8[]                 layers  # 몇 층인가. 0이 맨 아래.
