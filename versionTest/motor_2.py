"""
MDD3A Motor Driver — Single Motor Interactive Control
=====================================================
Wiring:
  MDD3A M1A → RPi GPIO 16
  MDD3A M1B → RPi GPIO 20
  MDD3A GND → RPi GND
"""
import time
import os

os.system("sudo pkill pigpiod")
os.system("sudo pigpiod")
time.sleep(1)

import pigpio


# ── Pin Configuration ──
PIN_A    = 16
PIN_B = 20
PWM_FREQ = 1000  # Hz
# ──────────────────────

pwm = pigpio.pi()
if not pwm.connected:
    print("Could not connect to pigpio daemon")
    exit(1)

pwm.set_mode(PIN_A, pigpio.OUTPUT)
pwm.set_mode(PIN_B, pigpio.OUTPUT)
pwm.set_PWM_dutycycle(PIN_A, 0)
pwm.set_PWM_dutycycle(PIN_B, 0)

pwm.set_PWM_frequency(PIN_A, PWM_FREQ)
pwm.set_PWM_frequency(PIN_B, PWM_FREQ)


def runMotor(speed, direction):
    """
    MDD3A Truth Table (pigpio):
      Forward  (direction='f') → PIN_A=PWM, PIN_B=0
      Backward (direction='b') → PIN_A=0,   PIN_B=PWM
      Stop     (direction='s') → PIN_A=0,   PIN_B=0
    """
    duty = int(max(0.0, min(100.0, speed)) * 2.55)  # 0–100 → 0–255
    print(f"duty: {duty}")
    if direction == "f":        # forward
        pwm.set_PWM_dutycycle(PIN_B, 0)
        pwm.set_PWM_dutycycle(PIN_A, duty)
    elif direction == "b":      # backward
        pwm.set_PWM_dutycycle(PIN_A, 0)
        pwm.set_PWM_dutycycle(PIN_B, duty)
    else:                       # stop / brake
        pwm.set_PWM_dutycycle(PIN_A, 0)
        pwm.set_PWM_dutycycle(PIN_B, 0)


def get_direction() -> str:
    while True:
        d = input("Direction [f=forward | b=backward | s=stop]: ").strip().lower()
        if d in ("f", "b", "s"):
            return d
        print("  ✗ Invalid. Enter f, b, or s.")


def get_speed() -> float:
    while True:
        try:
            s = float(input("Speed [0-100]: ").strip())
            if 0 <= s <= 100:
                return s
            print("  ✗ Speed must be between 0 and 100.")
        except ValueError:
            print("  ✗ Enter a numeric value.")


def main():
    prev_speed = 0
    print("================================")
    print("  MDD3A Single Motor Control")
    print("  Ctrl+C to exit")
    print("================================\n")
    try:
        direction = get_direction()
        speed = get_speed()
        while True:

            total_power = 0.001*speed + 0.999*prev_speed
            runMotor(total_power, direction)
            print(f"power = {total_power}")
            prev_speed = total_power
            if direction == "s":
                runMotor(0, "s")
                print("  ✔ Motor stopped.\n")
            else:
                
                

                '''label = "Forward" if direction == "f" else "Backward"
                print(f"  ✔ Motor running {label} at {total_power:.1f}%\n")'''

    except KeyboardInterrupt:
        print("\n[Ctrl+C detected]")

    finally:
        print("Stopping motor and cleaning up...")
        pwm.set_PWM_dutycycle(PIN_A, 0)
        pwm.set_PWM_dutycycle(PIN_B, 0)
        pwm.stop()
        print("Done.")


if __name__ == "__main__":
    main()
