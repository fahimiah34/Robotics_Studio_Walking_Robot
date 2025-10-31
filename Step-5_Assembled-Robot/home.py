## Code to boot, move each leg, and then home

from pylx16a.lx16a import *
import time

# ---------- Settings ----------
PORT = "/dev/ttyUSB0"
SERIAL_TIMEOUT_S = 0.03

NEUTRAL = 90
THIGH_TARGET = 120
SHANK_TARGET = 45         # per-leg sequence target for shank

FINAL_THIGH = 120
FINAL_SHANK = 30          # <-- changed per your request

MOVE_MS = 900             # per-step move time
NEUTRAL_MS = 700          # time to move to neutral at the start
PAUSE_S = 0.3             # cushion after each move

# Leg order: Top-Right -> Top-Left -> Back-Right -> Back-Left
LEG_ORDER = [
    ("TR", 11, 12),
    ("TL", 21, 22),
    ("BR", 31, 32),
    ("BL", 41, 42),
]

def move_pair(thigh_servo, shank_servo, thigh_angle, shank_angle, duration_ms):
    """Move a leg's thigh & shank together over duration_ms."""
    thigh_servo.move(thigh_angle, time=duration_ms, relative=False, wait=False)
    shank_servo.move(shank_angle, time=duration_ms, relative=False, wait=False)

def sequence_one_leg(name, thigh_servo, shank_servo):
    """For a single leg: go to (120,45), then back to (90,90)."""
    print(f"\n[{name}] -> (thigh={THIGH_TARGET}, shank={SHANK_TARGET})")
    move_pair(thigh_servo, shank_servo, THIGH_TARGET, SHANK_TARGET, MOVE_MS)
    time.sleep(MOVE_MS/1000.0 + PAUSE_S)

    print(f"[{name}] -> back to neutral (90,90)")
    move_pair(thigh_servo, shank_servo, NEUTRAL, NEUTRAL, MOVE_MS)
    time.sleep(MOVE_MS/1000.0 + PAUSE_S)

def final_all_together(servos):
    """All legs together: thigh=120, shank=30 (queued start for synch)."""
    print("\n[ALL] -> final pose (thigh=120, shank=30)")
    # Queue all moves first (wait=True), then start them together
    for sid, s in servos.items():
        if sid % 10 == 1:  # thigh IDs: 11,21,31,41 (end with 1)
            s.move(FINAL_THIGH, time=MOVE_MS, relative=False, wait=True)
        else:              # shank IDs: 12,22,32,42 (end with 2)
            s.move(FINAL_SHANK, time=MOVE_MS, relative=False, wait=True)
    for s in servos.values():
        s.move_start()
    time.sleep(MOVE_MS/1000.0 + PAUSE_S)

def main():
    LX16A.initialize(PORT, SERIAL_TIMEOUT_S)

    # Create servo objects
    servos = {}
    for label, thigh_id, shank_id in LEG_ORDER:
        try:
            servos[thigh_id] = LX16A(thigh_id)
            servos[shank_id] = LX16A(shank_id)
        except ServoTimeoutError as e:
            print(f"Init error: servo {e.id_} not responding. Check wiring/power/ID.")
            return

    # Ensure we're starting from neutral (90,90) for all 8 joints
    print("Moving all joints to neutral (90°) to start...")
    for sid, s in servos.items():
        s.move(NEUTRAL, time=NEUTRAL_MS, relative=False, wait=False)
    time.sleep(NEUTRAL_MS/1000.0 + PAUSE_S)

    # Run the sequence: TR -> TL -> BR -> BL
    for name, thigh_id, shank_id in LEG_ORDER:
        sequence_one_leg(name, servos[thigh_id], servos[shank_id])

    # Finish: all legs together to (thigh=120, shank=30)
    final_all_together(servos)

    print("✅ Sequence complete.")

if __name__ == "__main__":
    main()
