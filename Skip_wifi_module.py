import pigpio
import socket
import time
import threading
import signal
import sys

# Setup PWM (38kHz)
PWM_PIN = 18
FREQUENCY = 38000
DUTY_CYCLE = 300000

# LED Setup
LED_PIN = 19
STATUS_LED =  26

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    print("Not connected")
    exit(1)

# Setup LED pin as output
pi.set_mode(LED_PIN, pigpio.OUTPUT)
pi.set_mode(STATUS_LED, pigpio.OUTPUT)
pi.write(LED_PIN, 0)  # Start with LED OFF
pi.write(STATUS_LED, 1)

# Setup UDP Socket
HOST = '172.20.10.9'
PORT = 12345

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((HOST, PORT))

# State tracking variables
current_led_state = 0  # Start with LED OFF
running = True

# Handle cleanup on exit
def cleanup(signum, frame):
    global running
    print("\nExiting... Cleaning up resources.")

    # Stop PWM and turn off LED
    pi.hardware_PWM(PWM_PIN, 0, 0)
    pi.write(LED_PIN, 0)
    pi.writre(STATUS_LED, 0)

    # Close socket
    sock.close()

    # Stop pigpio
    pi.stop()

    running = False
    sys.exit(0)

# Register signals for Ctrl+C (SIGINT) and termination (SIGTERM)
signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

# Generate PWM signal (38kHz)
def generate_pwm_signal():
    try:
        print(f"Starting {FREQUENCY}Hz PWM on GPIO {PWM_PIN}...")
        pi.hardware_PWM(PWM_PIN, FREQUENCY, DUTY_CYCLE)
    except Exception as e:
        print(f"PWM Error: {e}")

# Listen for UDP messages and update LED state
def listen_for_udp():
    global current_led_state
    print(f"Listening for UDP messages on {HOST}:{PORT}")

    while running:
        sock.settimeout(None)  # Block until a message is received
        try:
            data, addr = sock.recvfrom(1024)
            message = data.decode('utf-8').strip()
            print(f"Received {message} from {addr}")

            if message == '1':
                current_led_state = 1  # Keep LED ON
            elif message == '0':
                current_led_state = 0  # Keep LED OFF

            # Respond to sender (optional)
            sock.sendto(b"Message received", addr)

        except Exception as e:
            print(f"UDP Error: {e}")

# Monitor LED state and apply changes
def monitor_led():
    global current_led_state
    while running:
        pi.write(LED_PIN, current_led_state)  # Set LED to last received value
        time.sleep(0.1)  # Check every 100ms

generate_pwm_signal()


# Create threads
#pwm_thread = threading.Thread(target=generate_pwm_signal)
udp_thread = threading.Thread(target=listen_for_udp)
led_monitor_thread = threading.Thread(target=monitor_led)

# Start threads
#pwm_thread.start()
udp_thread.start()
led_monitor_thread.start()

# Wait for threads to finish
#pwm_thread.join()
udp_thread.join()
led_monitor_thread.join()
