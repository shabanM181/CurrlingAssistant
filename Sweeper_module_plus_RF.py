import pigpio
import spidev
import RPi.GPIO as GPIO
import time
import threading
import signal
import sys

# --- LED Setup ---
LED_PIN = 4
STATUS_LED = 26
# --- RF Module Setup ---
CE_PIN = 22
CSN_PIN = 8
IR_GPIO = 18

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
running = True
msg = ""
rf_thread = None
led_monitor_thread = None

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
GPIO.setup(IR_GPIO, GPIO.IN)
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
    global msg
    while running:
        status = read_register(STATUS)
        if status & 0x40:  # RX_DR bit
            data = spi.xfer2([CMD_R_RX_PAYLOAD] + [0x00] * 32)
            spi.xfer2([CMD_FLUSH_RX])
            spi.xfer2([CMD_W_REGISTER | STATUS, 0x40])  # Clear RX_DR

            received_msg = "".join(chr(b) for b in data[1:] if b != 0).strip()
            print(f"RF Received: {received_msg}")
            msg = received_msg
        time.sleep(0.1)

def monitor_led():
    current_led_state = 0
    while running:
        IR_STATE = GPIO.input(IR_GPIO)
        if msg == "Pass!" or IR_STATE == GPIO.HIGH:
            print("IR_GPIO: ", IR_STATE)
            current_led_state = 0
        elif msg == "Fail!" and IR_STATE == GPIO.LOW:
            current_led_state = 1
            print("IR_GPIO: ", IR_STATE)

        pi.write(LED_PIN, current_led_state)
        time.sleep(2)
        pi.write(LED_PIN, 0)

# --- Cleanup ---
def cleanup(signum, frame):
    global running
    print("\nExiting... Cleaning up resources.")
    running = False

    if rf_thread is not None:
        rf_thread.join()
    if led_monitor_thread is not None:
        led_monitor_thread.join()

    pi.write(LED_PIN, 0)
    pi.write(STATUS_LED, 0)
    pi.stop()
    spi.close()
    GPIO.cleanup()
    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

# --- Start Program ---
init_nrf24()

rf_thread = threading.Thread(target=receive_rf_data, daemon=True)
led_monitor_thread = threading.Thread(target=monitor_led, daemon=True)

rf_thread.start()
led_monitor_thread.start()

rf_thread.join()
led_monitor_thread.join()
