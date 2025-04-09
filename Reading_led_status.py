import RPi.GPIO as GPIO
import time

IR_PIN = 16

GPIO.setmode(GPIO.BCM)
GPIO.setup(IR_PIN, GPIO.IN)

try:
        while True:
                state = GPIO.input(IR_PIN)
                #print(f"IR Reciver State: {'DETECTED}' if state == 0 else 'NOT DETECTED'}")
                if state == 1:
                        print("IR Signal not Detected")
                else:
                        print("IR Signal Detected")
                time.sleep(0.5)
except KeyboardInterrupt:
        print("Existing...")
finally:
        GPIO.cleanup()
