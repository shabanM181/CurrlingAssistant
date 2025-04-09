import spidev
import RPi.GPIO as GPIO
import time
import sys

# GPIO setup
CE_PIN = 22
INPUT_PIN = 18
STATUS_LED = 26

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(CE_PIN, GPIO.OUT)
GPIO.setup(INPUT_PIN, GPIO.IN)

# SPI setup
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000

# NRF24L01+ registers
CONFIG = 0x00
EN_AA = 0x01
EN_RXADDR = 0x02
SETUP_AW = 0x03
SETUP_RETR = 0x04
RF_CH = 0x05
RX_PW_P0 = 0x11
RF_SETUP = 0x06
STATUS = 0x07
TX_ADDR = 0x10
RX_ADDR_P0 = 0x0A
FIFO_STATUS = 0x17

# NRF24L01+ commands
R_REGISTER = 0x00
W_REGISTER = 0x20
FLUSH_TX = 0xE1
W_TX_PAYLOAD = 0xA0

GPIO.setup(STATUS_LED, GPIO.OUT)
GPIO.output(STATUS_LED, GPIO.HIGH)

def write_register(register, value):
    GPIO.output(CE_PIN, GPIO.LOW)
    if isinstance(value, list):
        spi.xfer2([W_REGISTER | register] + value)
    else:
        spi.xfer2([W_REGISTER | register, value])
    GPIO.output(CE_PIN, GPIO.HIGH)

def read_register(register):
    GPIO.output(CE_PIN, GPIO.LOW)
    response = spi.xfer2([R_REGISTER | register, 0])
    GPIO.output(CE_PIN, GPIO.HIGH)
    return response[1]

def send_data(data):
    spi.xfer2([FLUSH_TX])  # Clear TX FIFO
    payload = [W_TX_PAYLOAD] + list(data) + [0x00] * (32 - len(data))
    print(f"Sending payload: {payload}")
    spi.xfer2(payload)
    time.sleep(0.0001)

def wait_for_transmit():
    timeout = time.time() + 1
    while time.time() < timeout:
        status = read_register(STATUS)
        if status & 0x20:  # TX_DS
            spi.xfer2([W_REGISTER | STATUS, 0x20])  # Clear TX_DS
            return True
        elif status & 0x10:  # MAX_RT
            print("Max retries reached. Flushing TX...")
            spi.xfer2([FLUSH_TX])
            spi.xfer2([W_REGISTER | STATUS, 0x10])  # Clear MAX_RT
            return False
    print("Transmission timeout.")
    return False

def init_nrf24l01():
    write_register(CONFIG, 0x0E)
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

def main():
    init_nrf24l01()
    print("Monitoring GPIO pin for changes...")

    last_state = None
    try:
        while True:
            current_state = GPIO.input(INPUT_PIN)
            if current_state != last_state:
                message = "Pass!" if current_state == GPIO.LOW else "Fail!"
                print(f"GPIO {'HIGH' if current_state else 'LOW'}: Sending '{message}'")
                send_data(message.encode())
                wait_for_transmit()
                last_state = current_state
            time.sleep(0.1)
    except KeyboardInterrupt:
        GPIO.output(STATUS_LED, GPIO.LOW)
        spi.close()
        GPIO.cleanup()
        print("\nClean exit.")

if __name__ == "__main__":
    main()
