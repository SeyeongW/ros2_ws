import time
from camera import SiyiGimbal  # ← 네 파일 이름으로 수정

g = SiyiGimbal(debug=True)   # 디버깅 로그 출력됨
time.sleep(0.5)

print("Setting LOCK MODE...")
g.set_lock_mode()
time.sleep(1.0)

print("Yaw Right")
g.send_speed(30, 0)
time.sleep(1)

print("Stop")
g.send_speed(0, 0)
time.sleep(0.5)

print("Yaw Left")
g.send_speed(-30, 0)
time.sleep(1)

print("Stop")
g.send_speed(0, 0)