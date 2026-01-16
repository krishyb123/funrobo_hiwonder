import time
from ros_robot_controller_sdk import Board

# ---------------- CONFIG ----------------
SERVO_ID = 3          # change this
START = 500           # start near center
STEP = 5              # smaller = safer
DELAY = 0.05          # seconds between steps
# ----------------------------------------

board = Board()

def pulse_to_angle(pulse):
    return pulse * 240.0 / 1000.0

def move(pulse):
    board.bus_servo_set_position(0.1, [[SERVO_ID, pulse]])
    time.sleep(DELAY)

print("\n=== LX-15D CALIBRATION ===")
print("Servo ID:", SERVO_ID)
print("Watch the joint carefully.")
print("Ctrl+C IMMEDIATELY if it approaches a hard stop.\n")


board.bus_servo_set_position(2, [[SERVO_ID, START]])
time.sleep(2)

# ---- Sweep forward ----
print("Sweeping FORWARD...")
pulse = START
try:
    while pulse <= 1000:
        move(pulse)
        angle = pulse_to_angle(pulse)
        print(f"Pulse: {pulse:4d} | Angle: {angle:6.1f}°", end="\r")
        pulse += STEP
except KeyboardInterrupt:
    max_pulse = pulse
    print(f"\nStopped at pulse = {max_pulse} ({pulse_to_angle(max_pulse):.1f}°)")
max_pulse = pulse

# ---- Pause ----
time.sleep(1.0)

# ---- Sweep backward ----
print("\nSweeping BACKWARD...")
pulse = START
try:
    while pulse >= 0:
        move(pulse)
        angle = pulse_to_angle(pulse)
        print(f"Pulse: {pulse:4d} | Angle: {angle:6.1f}°", end="\r")
        pulse -= STEP
except KeyboardInterrupt:
    min_pulse = pulse
    print(f"\nStopped at pulse = {min_pulse} ({pulse_to_angle(min_pulse):.1f}°)")

board.bus_servo_set_position(1, [[1, 500], [2, 500], [3, 500], [4, 500], [5, 500], [6, 500]])

print("\n=== CALIBRATION RESULT ===")
print("Recommended limits:")
print(f"  min_pulse = {pulse} ({pulse_to_angle(pulse):.1f}°)")
print(f"  max_pulse = {max_pulse} ({pulse_to_angle(max_pulse):.1f}°)")

