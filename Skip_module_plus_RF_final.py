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
CAVI@CAVI:~/dev/CAVI $ ls
38khz_plus_udp.py  hardwarePWM.py  IR_LED_38kHz.py        lib_nrf24  RF24      test.py   untitled.py
adafruit           IR_LED_38kHz    IR_reading_voltage.py  march7.py  serviced  test_spi
CAVI@CAVI:~/dev/CAVI $ cd test_spi/
CAVI@CAVI:~/dev/CAVI/test_spi $ ls
abdul_version.py    moath_version.py  receiver_wifi.py   Skips_38khz_NRF240.py  TX_NRF240.py  wifi_client.py
emitter_working.py  __pycache__       Rx_new_March11.py  test.py                udp.py
lib_nrf24.py        py-spidev-master  RX_NRF240.py       test_spi.py            venv
CAVI@CAVI:~/dev/CAVI/test_spi $ cat moath_version.py
import pigpio
import spidev
import RPi.GPIO as GPIO
import time
import threading
import signal
import sys

# --- PWM Setup (38kHz) ---
PWM_PIN = 18
FREQUENCY = 38000
DUTY_CYCLE = 300000

# --- LED Setup ---
LED_PIN = 4
STATUS_LED = 26

# --- RF Module Setup ---
CE_PIN = 22
CSN_PIN = 8

# NRF24L01+ commands and registers
CMD_R_REGISTER = 0x00
CMD_W_REGISTER = 0x20
CMD_R_RX_PAYLOAD = 0x61
CMD_FLUSH_RX = 0xE2
STATUS = 0x07
RX_PW_P0 = 0x11
EN_AA = 0x01
EN_RXADDR = 0x02
SETUP_AW = 0x03
SETUP_RETR = 0x04
RF_CH = 0x05
RF_SETUP = 0x06
CONFIG = 0x00
TX_ADDR = 0x10
RX_ADDR_P0 = 0x0A

# --- Global Variables ---
last_led_state = None
current_led_state = 0
running = True

# --- Initialize pigpio ---
pi = pigpio.pi()
if not pi.connected:
    print("Not connected to pigpio")
    sys.exit(1)
pi.set_mode(LED_PIN, pigpio.OUTPUT)
pi.set_mode(STATUS_LED, pigpio.OUTPUT)
pi.write(LED_PIN, 0)
pi.write(STATUS_LED, 1)

# --- SPI and GPIO Setup for RF ---
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000

GPIO.setmode(GPIO.BCM)
GPIO.setup(CE_PIN, GPIO.OUT)
GPIO.output(CE_PIN, GPIO.LOW)

# --- NRF24 Functions ---
def read_register(register):
    return spi.xfer2([CMD_R_REGISTER | register, 0x00])[1]

def write_register(register, value):
    GPIO.output(CE_PIN, GPIO.LOW)
    if isinstance(value, list):
        spi.xfer2([CMD_W_REGISTER | register] + value)
    else:
        spi.xfer2([CMD_W_REGISTER | register, value])
    GPIO.output(CE_PIN, GPIO.HIGH)

def init_nrf24():
    print("Initializing NRF24L01+...")
    spi.xfer2([CMD_W_REGISTER | CONFIG, 0x0F])  # Power up, PRIM_RX
    write_register(EN_AA, 0x01)
    write_register(EN_RXADDR, 0x01)
    write_register(SETUP_AW, 0x03)
    write_register(SETUP_RETR, 0x1A)
    write_register(RF_CH, 0x6C)
    write_register(RF_SETUP, 0x07)
    write_register(TX_ADDR, [0xE7] * 5)
    write_register(RX_ADDR_P0, [0xE7] * 5)
    write_register(RX_PW_P0, 32)
    GPIO.output(CE_PIN, GPIO.HIGH)
    print("NRF24L01+ Ready (Receiver Mode)")

def receive_rf_data():
    global current_led_state
    while running:
        status = read_register(STATUS)
        if status & 0x40:  # RX_DR bit
            data = spi.xfer2([CMD_R_RX_PAYLOAD] + [0x00] * 32)
            spi.xfer2([CMD_FLUSH_RX])
            spi.xfer2([CMD_W_REGISTER | STATUS, 0x40])  # Clear RX_DR

            msg = "".join(chr(b) for b in data[1:] if b != 0).strip()
            print(f"RF Received: {msg}")

            if msg == "Pass!":
                current_led_state = 1
            elif msg == "Fail!":
                    current_led_state = 0

        time.sleep(0.1)

# --- Cleanup ---
def cleanup(signum, frame):
    global running
    print("\nExiting... Cleaning up resources.")
    pi.hardware_PWM(PWM_PIN, 0, 0)
    pi.write(LED_PIN, 0)
    pi.write(STATUS_LED, 0)
    pi.stop()
    spi.close()
    GPIO.cleanup()
    running = False
    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

# --- Threads ---
def generate_pwm_signal():
    try:
        print(f"Starting {FREQUENCY}Hz PWM on GPIO {PWM_PIN}...")
        pi.hardware_PWM(PWM_PIN, FREQUENCY, DUTY_CYCLE)
        while running:
            time.sleep(1)
    except Exception as e:
        print(f"PWM Error: {e}")

def monitor_led():
    global current_led_state
    global last_led_state
    while running:
        if last_led_state != current_led_state:
            pi.write(LED_PIN, current_led_state)

# --- Start Program ---
init_nrf24()

pwm_thread = threading.Thread(target=generate_pwm_signal)
rf_thread = threading.Thread(target=receive_rf_data)
led_monitor_thread = threading.Thread(target=monitor_led)

pwm_thread.start()
rf_thread.start()
led_monitor_thread.start()

pwm_thread.join()
rf_thread.join()
led_monitor_thread.join()
