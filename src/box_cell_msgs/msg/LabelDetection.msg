# 상면 라벨 한 건의 판독 결과. 기획서 5.7의 정의 그대로다.
#
# 파이프라인 : 촬영 -> QR 디코드 -> 네 모서리 좌표 -> 호모그래피 -> (x, y, yaw)
# 라벨이 상면 중앙에 붙으므로 모서리 평균이 곧 박스 중심이고,
# 변의 기울기가 회전각이다. 별도 검출 모델이 필요 없다.

std_msgs/Header header

string  code            # QR/바코드 문자열. MES 품목 조회 키.
bool    ok              # 디코드 성공 여부. false면 아래 값은 의미 없다.
float64 center_x        # 박스 중심 x. 상판(table) 좌표계, m.
float64 center_y        # 박스 중심 y. 상판(table) 좌표계, m.
float64 yaw             # 회전각, rad. 박스가 60 각 정사각형이라 90도 대칭이므로
                        # ±45도 안으로 정규화한다. FR5 J6 범위 ±175도에 여유가 크다.
float64 confidence      # 0..1. 디코드 여유와 모서리 잔차에서 낸다.

uint8   attempt         # 몇 번째 시도인가. 3회 실패면 예외 통으로 간다.
string  source          # 어느 카메라인가. c1_conveyor 또는 c4_wrist.
