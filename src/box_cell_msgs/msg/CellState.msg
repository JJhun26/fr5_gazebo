# task_manager 상태 기계의 현재 상태. 트윈 화면과 대시보드가 이걸 본다.
#
# IDLE -> WAIT_BOX -> STOPPED -> READ -> QUERY -> PLAN -> EXECUTE -> RECORD -> IDLE
# READ 실패 3회 -> EXCEPTION -> 예외 통 이송 -> IDLE
# RECORD에서 팔레트 가득 -> DEPALLETIZE -> 반출 후 대상 팔레트 전환 -> IDLE

std_msgs/Header header

uint8 IDLE=0
uint8 WAIT_BOX=1
uint8 STOPPED=2
uint8 READ=3
uint8 QUERY=4
uint8 PLAN=5
uint8 EXECUTE=6
uint8 RECORD=7
uint8 EXCEPTION=8
uint8 DEPALLETIZE=9

uint8   state
string  state_name
string  detail          # 사람이 읽는 한 줄. 화면에 그대로 띄운다.
uint8   source_pallet   # 지금 꺼내는 쪽
uint8   target_pallet   # 지금 쌓는 쪽
uint32  cycle_count     # 기동 후 처리한 건수
float64 last_cycle_sec  # 직전 한 사이클 시간. 목표는 20초.
uint32  exception_count
