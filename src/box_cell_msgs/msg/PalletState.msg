# 팔레트 한 장의 점유 기록. pallet_manager가 들고 있는 것과 같다.

uint8   pallet_id
bool[]  occupied        # 길이 8. 인덱스가 곧 슬롯 번호다.
string[] codes          # 슬롯별 라벨 문자열. 비었으면 "".
uint8   count
bool    full
