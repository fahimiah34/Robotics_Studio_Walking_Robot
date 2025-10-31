# Code to boot the robot so all legs are stright up

from pylx16a.lx16a import *
import time

PORT = "/dev/ttyUSB0"
SERIAL_TIMEOUT_S = 0.03
TARGET_ANGLE = 90       # your "legs straight up" position
DURATION_MS = 1000      # move time in milliseconds (1 s)
IDS = [11, 12, 21, 22, 31, 32, 41, 42]

# 1. Initialize the bus
LX16A.initialize(PORT, SERIAL_TIMEOUT_S)

# 2. Connect to each servo
servos = []
for i in IDS:
    try:
        s = LX16A(i)
        servos.append(s)
    except ServoTimeoutError:
        print(f"[ID {i}] not responding.")
    except ServoError as e:
        print(f"[ID {i}] init error: {e}")

# 3. Print current angles
print("\nCurrent servo angles:")
for s in servos:
    try:
        ang = s.get_physical_angle()
        print(f"[ID {s.get_id()}]  {ang:.2f}°")
    except ServoError as e:
        print(f"[ID {s.get_id()}] read error: {e}")

# 4. Move all to 90° together
print("\nMoving all servos to 90° ...")
for s in servos:
    try:
        s.move(TARGET_ANGLE, time=DURATION_MS, relative=False, wait=False)
    except ServoError as e:
        print(f"[ID {s.get_id()}] move error: {e}")

# 5. Wait for motion to complete
time.sleep(DURATION_MS/1000.0 + 0.5)

print("✅ Done. All servos commanded to 90°.")
