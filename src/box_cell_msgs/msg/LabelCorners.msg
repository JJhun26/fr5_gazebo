# QR 디코드 결과의 원시 형태. label_reader가 내고 pose_resolver가 받는다.
#
# 두 노드를 나눈 이유는 좌표계다. label_reader는 픽셀만 알고, 실세계 mm는
# 모른다. 호모그래피와 카메라 외부 파라미터는 pose_resolver가 쥔다.
# 하드웨어가 오면 캘리브레이션이 바뀌는데, 그때 고치는 곳이 한 군데여야 한다.

std_msgs/Header header

string  code            # 디코드된 문자열. 실패면 빈 문자열.
bool    ok
float64[8] corners      # 픽셀 좌표 (x0,y0, x1,y1, x2,y2, x3,y3). QR 기준 시계방향.
float64 decode_margin   # 디코드 여유. 신뢰도 계산에 쓴다.
uint8   attempt
string  source          # c1_conveyor 또는 c4_wrist
