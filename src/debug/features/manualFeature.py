from machine import UART, Pin, I2C
import time
import network
import urequests

# ---------------- WiFi Setup ----------------
CAMERA_SSID = "ESP32_CAM_AP"
CAMERA_PASSWORD = "12345678"
CAMERA_IP = "192.168.4.1"

wlan = network.WLAN(network.STA_IF)
wlan.active(True)

def connect_to_camera():
    """Connect to ESP32-CAM access point"""
    if not wlan.isconnected():
        print(f"Connecting to {CAMERA_SSID}...")
        wlan.connect(CAMERA_SSID, CAMERA_PASSWORD)
        
        timeout = 10
        while not wlan.isconnected() and timeout > 0:
            print(".", end="")
            time.sleep(1)
            timeout -= 1
        
        if wlan.isconnected():
            print(f"\nConnected! IP: {wlan.ifconfig()[0]}")
            return True
        else:
            print("\nConnection failed!")
            return False
    return True

def send_camera_command(command):
    """Send command to camera (start_record or stop_record)"""
    try:
        url = f"http://{CAMERA_IP}/command"
        response = urequests.post(url, data=command)
        result = response.text
        response.close()
        print(f"Camera response: {result}")
        return True
    except Exception as e:
        print(f"Camera command error: {e}")
        return False

# ---------------- LED Indicator ----------------
# Built-in LED (usually GPIO48 on ESP32-S3, change if different)
LED_PIN = 48
led = Pin(LED_PIN, Pin.OUT)
led.value(0)  # Off initially

# LED blink state for recording indicator
led_state = False
last_led_blink = time.ticks_ms()

# ---------------- iBus setup ----------------
ibus_uart = UART(2, baudrate=115200, tx=Pin(36), rx=Pin(35))
PACKET_SIZE = 32
buffer = bytearray()

def read_ibus_packet():
    global buffer
    while ibus_uart.any():
        b = ibus_uart.read(ibus_uart.any())
        if b:
            buffer.extend(b)
            if len(buffer) > 64:
                buffer = buffer[-64:]

    while len(buffer) >= PACKET_SIZE:
        if buffer[0] == 0x20 and buffer[1] == 0x40:
            packet = buffer[:PACKET_SIZE]
            buffer = buffer[PACKET_SIZE:]
            return packet
        else:
            buffer = buffer[1:]
    return None

def parse_ibus_channels(packet):
    return [(packet[2+i*2] | (packet[3+i*2] << 8)) for i in range(14)]

# ---------------- PCA9685 on I2C ----------------
i2c = I2C(0, scl=Pin(2), sda=Pin(1))
PCA9685_ADDR = 0x40
MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE

def write_byte(reg, val):
    i2c.writeto_mem(PCA9685_ADDR, reg, bytes([val]))

def set_pwm(ch, on, off):
    base = 0x06 + 4*ch
    i2c.writeto_mem(PCA9685_ADDR, base, bytes([on & 0xFF]))
    i2c.writeto_mem(PCA9685_ADDR, base+1, bytes([on >> 8]))
    i2c.writeto_mem(PCA9685_ADDR, base+2, bytes([off & 0xFF]))
    i2c.writeto_mem(PCA9685_ADDR, base+3, bytes([off >> 8]))

# Init PCA9685
write_byte(MODE1, 0x00)
write_byte(MODE2, 0x04)
FREQ_HZ = 50
RESOLUTION = 4096
prescale = int(round(25000000 / (RESOLUTION * FREQ_HZ)) - 1)
oldmode = i2c.readfrom_mem(PCA9685_ADDR, MODE1, 1)[0]
write_byte(MODE1, (oldmode & 0x7F) | 0x10)
write_byte(PRESCALE, prescale)
write_byte(MODE1, oldmode)
time.sleep_ms(5)
write_byte(MODE1, oldmode | 0x80)

# ---------------- ESC Setup ----------------
LEFT_ESC = 1
RIGHT_ESC = 3
MIN_PULSE = 1000
MAX_PULSE = 2000

def pulse_us_to_counts(pulse_us):
    period_us = 1_000_000 / FREQ_HZ
    return int((pulse_us / period_us) * RESOLUTION)

def set_motor(ch, pulse):
    pulse = max(MIN_PULSE, min(MAX_PULSE, pulse))
    set_pwm(ch, 0, pulse_us_to_counts(pulse))

# ---------------- Mixing ----------------
def mix(throttle_raw, steer_raw):
    throttle = (throttle_raw - 1000) / 1000.0
    steer = (steer_raw - 1500) / 500.0

    left = throttle * (1 + steer)
    right = throttle * (1 - steer)

    left = max(0, min(1, left))
    right = max(0, min(1, right))

    return (
        int(MIN_PULSE + left * (MAX_PULSE - MIN_PULSE)),
        int(MIN_PULSE + right * (MAX_PULSE - MIN_PULSE))
    )

# ---------------- Startup ----------------
set_motor(LEFT_ESC, MIN_PULSE)
set_motor(RIGHT_ESC, MIN_PULSE)

# Connect to camera
print("Connecting to ESP32-CAM...")
connect_to_camera()

time.sleep(2)

# ---------------- Stepper Setup ----------------
STEP_PIN = 7
DIR_PIN  = 6
EN_PIN   = 5

is_centered = False
last_mode = "unknown"
CENTER_TOLERANCE = 1

step = Pin(STEP_PIN, Pin.OUT)
dir  = Pin(DIR_PIN, Pin.OUT)
en   = Pin(EN_PIN, Pin.OUT)

en.value(0)

STEPS_PER_REV = 200
DEG_PER_STEP = 1.8
PAN_LIMIT_DEG = 180
PAN_STEPS = int(PAN_LIMIT_DEG / DEG_PER_STEP)
step_delay_us = 1500

current_step = -PAN_STEPS
direction = 1
last_step_time = time.ticks_us()

def update_stepper(mode):
    global current_step, direction, last_step_time
    global is_centered, last_mode

    if mode != last_mode:
        if mode == "autonomous":
            is_centered = False
        last_mode = mode

    if mode != "autonomous":
        step.value(0)
        en.value(0)
        return

    en.value(0)

    now = time.ticks_us()
    if time.ticks_diff(now, last_step_time) < step_delay_us * 2:
        return

    last_step_time = now

    if not is_centered:
        if abs(current_step) <= CENTER_TOLERANCE:
            current_step = 0
            direction = 1
            is_centered = True
            return

        dir.value(0 if current_step > 0 else 1)
        step.value(1)
        time.sleep_us(10)
        step.value(0)
        current_step += -1 if current_step > 0 else 1
        return

    dir.value(1 if direction == 1 else 0)
    step.value(1)
    time.sleep_us(10)
    step.value(0)
    current_step += direction

    if current_step >= PAN_STEPS:
        direction = -1
    elif current_step <= -PAN_STEPS:
        direction = 1

# ---------------- Recording State ----------------
is_recording = False
last_record_state = False

def update_recording_led():
    """Blink LED when recording, solid when not"""
    global led_state, last_led_blink
    
    if is_recording:
        # Blink at 2Hz (250ms on, 250ms off)
        if time.ticks_diff(time.ticks_ms(), last_led_blink) > 250:
            led_state = not led_state
            led.value(led_state)
            last_led_blink = time.ticks_ms()
    else:
        # LED off when not recording
        led.value(0)

# ---------------- Main Loop ----------------
last_rx = time.ticks_ms()
FAILSAFE_MS = 300
mode = "unknown"

print("\n=== System Ready ===")
print("Channel 7 controls recording:")
print("  >= 1900: START recording")
print("  <= 1100: STOP recording")

while True:
    packet = read_ibus_packet()
    if packet:
        ch = parse_ibus_channels(packet)
        last_rx = time.ticks_ms()

        # Mode from channel 6
        mode_ch = ch[6]
        if mode_ch <= 1100:
            mode = "autonomous"
        elif 1400 <= mode_ch <= 1600:
            mode = "hover"
        elif mode_ch >= 1900:
            mode = "manual"
        else:
            mode = "unknown"

        # Recording control from channel 7
        record_ch = ch[7]
        current_record_state = record_ch >= 1900
        
        # Detect state change
        if current_record_state != last_record_state:
            if current_record_state:
                # Start recording
                print("\n>>> RECORD SWITCH ON - Starting recording...")
                if send_camera_command("start_record"):
                    is_recording = True
                    print(">>> Recording STARTED")
                else:
                    print(">>> Failed to start recording!")
            else:
                # Stop recording
                print("\n>>> RECORD SWITCH OFF - Stopping recording...")
                if send_camera_command("stop_record"):
                    is_recording = False
                    print(">>> Recording STOPPED")
                else:
                    print(">>> Failed to stop recording!")
            
            last_record_state = current_record_state

        # Motor control
        throttle_raw = ch[2]
        steer_raw = ch[3]

        if mode == "manual":
            left, right = mix(throttle_raw, steer_raw)
        else:
            left, right = MIN_PULSE, MIN_PULSE

        set_motor(LEFT_ESC, left)
        set_motor(RIGHT_ESC, right)

        print(f"MODE: {mode:10s} | THR: {throttle_raw:4d} | STR: {steer_raw:4d} | L: {left:4d} | R: {right:4d} | REC: {'ON' if is_recording else 'OFF'} | CH7: {record_ch:4d}")

    # Failsafe
    if time.ticks_diff(time.ticks_ms(), last_rx) > FAILSAFE_MS:
        set_motor(LEFT_ESC, MIN_PULSE)
        set_motor(RIGHT_ESC, MIN_PULSE)

    # Update peripherals
    update_stepper(mode)
    update_recording_led()

    time.sleep_ms(20)
