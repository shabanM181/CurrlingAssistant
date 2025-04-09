import RPi.GPIO as GPIO
import socket
import time

# GPIO and Socket Setup
IR_PIN = 18
SERVER_IP = '172.20.10.9'  # Replace with Pi 2's IP address
PORT = 12345

GPIO.setmode(GPIO.BCM)
GPIO.setup(IR_PIN, GPIO.IN)

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    last_state = -1
    while True:
        state = GPIO.input(IR_PIN)

        if state != last_state:  # Only send if the state changes to reduce traffic
            last_state = state
            if state == 0:
                print("IR Signal Detected")
                sock.sendto(b'1', (SERVER_IP, PORT))  # Send '1' when IR detected
            else:
                print("IR Signal not Detected")
                sock.sendto(b'0', (SERVER_IP, PORT))  # Send '0' when IR not detected

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting...")
finally:
    GPIO.cleanup()
