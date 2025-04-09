import pigpio
import time

PWM_PIN = 18
FREQUENCY = 38000
DUTY_CYCLE = 300000

pi = pigpio.pi()
if not pi.connected:
        print("Not connected")
        exit(1)

try:
        print(f"Starting {FREQUENCY}Hz PWM on GPIO {PWM_PIN}...")
        pi.hardware_PWM(PWM_PIN, FREQUENCY, DUTY_CYCLE)

        time.sleep(180)

        pi.hardware_PWM(PWM_PIN, 0, 0)
        print("PWM stopped")

finally:
        pi.stop()
