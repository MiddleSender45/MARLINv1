from machine import UART, Pin, I2C
import time
import network
import urequests
import struct
import math
import os

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
LED_PIN = 48
led = Pin(LED_PIN, Pin.OUT)
led.value(0)

led_state = False
last_led_blink = time.ticks_ms()

# ---------------- iBus setup ----------------
ibus_uart = UART(2, baudrate=115115, tx=Pin(36), rx=Pin(35))
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

# ---------------- I2C Bus 0: PCA9685 ----------------
i2c = I2C(1, scl=Pin(2), sda=Pin(1))
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

# ---------------- I2C Bus 1: MPU6050 + HMC5883L ----------------
i2c_sensors = I2C(0, scl=Pin(9), sda=Pin(8), freq=100000)

# Helper function for two's complement
def twos_comp16(val):
    if val >= 32768:
        return val - 65536
    return val

# MPU6050 Setup
MPU6050_ADDR = 0x68

def init_mpu6050():
    try:
        # Wake up MPU6050
        i2c_sensors.writeto_mem(MPU6050_ADDR, 0x6B, b'\x00')
        time.sleep_ms(100)
        print("MPU6050 initialized")
        return True
    except Exception as e:
        print(f"MPU6050 init error: {e}")
        return False

def read_mpu6050():
    try:
        data = i2c_sensors.readfrom_mem(MPU6050_ADDR, 0x3B, 14)
        
        ax = twos_comp16(int.from_bytes(data[0:2], 'big')) / 16384.0
        ay = twos_comp16(int.from_bytes(data[2:4], 'big')) / 16384.0
        az = twos_comp16(int.from_bytes(data[4:6], 'big')) / 16384.0
        
        gx = twos_comp16(int.from_bytes(data[8:10], 'big')) / 131.0
        gy = twos_comp16(int.from_bytes(data[10:12], 'big')) / 131.0
        gz = twos_comp16(int.from_bytes(data[12:14], 'big')) / 131.0
        
        return {'ax': ax, 'ay': ay, 'az': az, 'gx': gx, 'gy': gy, 'gz': gz}
    except Exception as e:
        return None

# HMC5883L Setup with Calibration
HMC5883L_ADDR = 0x1E
MAG_OFFSET = [-128.5, 149.0, 120.0]  # Calibration offsets [X, Y, Z]
heading_filtered = 0
ALPHA = 0.15  # Filter coefficient

def init_hmc5883l():
    try:
        # Configuration
        i2c_sensors.writeto_mem(HMC5883L_ADDR, 0x00, b'\x70')  # 8-average, 15 Hz
        i2c_sensors.writeto_mem(HMC5883L_ADDR, 0x01, b'\xA0')  # Gain
        i2c_sensors.writeto_mem(HMC5883L_ADDR, 0x02, b'\x00')  # Continuous mode
        time.sleep_ms(100)
        print("HMC5883L initialized")
        return True
    except Exception as e:
        print(f"HMC5883L init error: {e}")
        return False

def read_mag_raw():
    """Read raw magnetometer data"""
    try:
        data = i2c_sensors.readfrom_mem(HMC5883L_ADDR, 0x03, 6)
        x = twos_comp16(int.from_bytes(data[0:2], 'big'))
        z = twos_comp16(int.from_bytes(data[2:4], 'big'))
        y = twos_comp16(int.from_bytes(data[4:6], 'big'))
        return x, y, z
    except Exception as e:
        return None

def get_heading():
    """Get calibrated and filtered heading"""
    global heading_filtered
    
    result = read_mag_raw()
    if result is None:
        return None
    
    mx, my, mz = result
    
    # Apply calibration offsets
    mx -= MAG_OFFSET[0]
    my -= MAG_OFFSET[1]
    
    # Calculate heading
    heading = math.degrees(math.atan2(my, mx))
    if heading < 0:
        heading += 360
    
    # Apply low-pass filter with wraparound handling
    diff = heading - heading_filtered
    if diff > 180:
        diff -= 360
    elif diff < -180:
        diff += 360
    
    heading_filtered += ALPHA * diff
    heading_filtered %= 360
    
    return heading_filtered

def heading_to_cardinal(h):
    """Convert heading to cardinal direction"""
    dirs = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    return dirs[int((h + 22.5) // 45) % 8]

# ---------------- GPS Setup ----------------
gps_uart = UART(1, baudrate=9600, tx=Pin(17), rx=Pin(18))
gps_buffer = ""
gps_last_data = time.ticks_ms()
GPS_TIMEOUT = 2000

def parse_gps():
    global gps_buffer, gps_last_data
    
    if gps_uart.any():
        try:
            data = gps_uart.read()
            if data:
                gps_last_data = time.ticks_ms()
                gps_buffer += data.decode('utf-8', 'ignore')
                
                while '\n' in gps_buffer:
                    line, gps_buffer = gps_buffer.split('\n', 1)
                    line = line.strip()
                    
                    if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                        parts = line.split(',')
                        if len(parts) > 9:
                            fix_quality = parts[6] if parts[6] else '0'
                            
                            if fix_quality != '0' and parts[2] and parts[4]:
                                lat_deg = float(parts[2][:2])
                                lat_min = float(parts[2][2:])
                                lat = lat_deg + lat_min / 60.0
                                if parts[3] == 'S':
                                    lat = -lat
                                
                                lon_deg = float(parts[4][:3])
                                lon_min = float(parts[4][3:])
                                lon = lon_deg + lon_min / 60.0
                                if parts[5] == 'W':
                                    lon = -lon
                                
                                sats = parts[7] if parts[7] else '0'
                                alt = parts[9] if parts[9] else '0'
                                
                                return {
                                    'status': 'FIX',
                                    'lat': lat,
                                    'lon': lon,
                                    'sats': int(sats),
                                    'alt': float(alt)
                                }
                            else:
                                sats = parts[7] if parts[7] else '0'
                                return {
                                    'status': 'NO_FIX',
                                    'sats': int(sats)
                                }
        except Exception as e:
            pass
    
    if time.ticks_diff(time.ticks_ms(), gps_last_data) > GPS_TIMEOUT:
        return {'status': 'NO_DATA'}
    
    return None

# ---------------- ESC Setup ----------------
LEFT_ESC = 3
RIGHT_ESC = 1
MIN_PULSE = 1000
MAX_PULSE = 2000
AUTO_MOTOR = 4
FULL_SPEED = MAX_PULSE

def pulse_us_to_counts(pulse_us):
    period_us = 1_000_000 / FREQ_HZ
    return int((pulse_us / period_us) * RESOLUTION)

def set_motor(ch, pulse):
    pulse = max(MIN_PULSE, min(MAX_PULSE, pulse))
    set_pwm(ch, 0, pulse_us_to_counts(pulse))

# ---------------- Heading Navigation ----------------
TARGET_HEADING = 210
HEADING_TOLERANCE = 5
BASE_SPEED = 1400
MAX_TURN_SPEED = 1700

def normalize_angle(angle):
    """Normalize angle to 0-360 range"""
    while angle < 0:
        angle += 360
    while angle >= 360:
        angle -= 360
    return angle

def angle_difference(current, target):
    """
    Calculate the shortest angular difference from current to target.
    Returns: -180 to +180 degrees
    Positive = turn right (clockwise)
    Negative = turn left (counter-clockwise)
    """
    diff = target - current
    while diff > 180:
        diff -= 360
    while diff < -180:
        diff += 360
    return diff

def calculate_turn_motors(current_heading, target_heading):
    """
    Calculate left and right motor speeds to turn toward target heading.
    Returns: (left_speed, right_speed)
    """
    error = angle_difference(current_heading, target_heading)
    
    # If within tolerance, stop motors
    if abs(error) < HEADING_TOLERANCE:
        return (MIN_PULSE, MIN_PULSE)
    
    # Calculate turn intensity (0.0 to 1.0)
    turn_intensity = min(abs(error) / 90.0, 1.0)
    
    # Calculate differential thrust
    if error > 0:
        # Turn right: increase LEFT, decrease RIGHT
        left_speed = int(BASE_SPEED + (MAX_TURN_SPEED - BASE_SPEED) * turn_intensity)
        right_speed = int(BASE_SPEED - (BASE_SPEED - MIN_PULSE) * turn_intensity * 0.5)
    else:
        # Turn left: increase RIGHT, decrease LEFT
        right_speed = int(BASE_SPEED + (MAX_TURN_SPEED - BASE_SPEED) * turn_intensity)
        left_speed = int(BASE_SPEED - (BASE_SPEED - MIN_PULSE) * turn_intensity * 0.5)
    
    left_speed = max(MIN_PULSE, min(MAX_PULSE, left_speed))
    right_speed = max(MIN_PULSE, min(MAX_PULSE, right_speed))
    
    return (left_speed, right_speed)

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

# ---------------- Telemetry Logging Setup ----------------
TELEMETRY_FILE = '/telemetry.csv'
LOG_INTERVAL_MS = 1000  # 1 sample per second
MAX_FILE_SIZE = 2 * 1024 * 1024  # 2 MB max file size
last_log_time = time.ticks_ms()

def init_telemetry():
    """Initialize telemetry file with headers - creates new file each session"""
    try:
        # Generate unique filename with timestamp
        session_id = time.ticks_ms()
        global TELEMETRY_FILE
        TELEMETRY_FILE = f'/telemetry_{session_id}.csv'
        
        # Create new file with headers
        with open(TELEMETRY_FILE, 'w') as f:
            f.write("time_ms,mode,heading,cardinal,target,error,left_motor,right_motor,auto_motor,gps_status,lat,lon,alt,sats,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,encoder_pos\n")
        
        print(f"Created new telemetry file: {TELEMETRY_FILE}")
        return True
    except Exception as e:
        print(f"Telemetry init error: {e}")
        return False
    
def log_telemetry(mode, heading_val, cardinal_val, motor_left, motor_right, motor_auto, mpu_data, gps_data, encoder_pos):
    """Log one line of telemetry data"""
    try:
        # Check file size before writing
        try:
            file_size = os.stat(TELEMETRY_FILE)[6]
            if file_size > MAX_FILE_SIZE:
                print("Max file size reached, starting new file...")
                os.remove(TELEMETRY_FILE)
                init_telemetry()
        except:
            pass
        
        timestamp = time.ticks_ms()
        heading_str = f"{heading_val:.1f}" if heading_val is not None else "N/A"
        cardinal_str = cardinal_val if cardinal_val else "N/A"
        error = angle_difference(heading_val, TARGET_HEADING) if heading_val is not None else 0
        
        # GPS data
        if gps_data and gps_data['status'] == 'FIX':
            gps_status = "FIX"
            lat = f"{gps_data['lat']:.6f}"
            lon = f"{gps_data['lon']:.6f}"
            alt = f"{gps_data['alt']:.1f}"
            sats = gps_data['sats']
        elif gps_data and gps_data['status'] == 'NO_FIX':
            gps_status = "NO_FIX"
            lat = "N/A"
            lon = "N/A"
            alt = "N/A"
            sats = gps_data['sats']
        else:
            gps_status = "NO_DATA"
            lat = "N/A"
            lon = "N/A"
            alt = "N/A"
            sats = 0
        
        # IMU data
        if mpu_data:
            acc_x = f"{mpu_data['ax']:.2f}"
            acc_y = f"{mpu_data['ay']:.2f}"
            acc_z = f"{mpu_data['az']:.2f}"
            gyro_x = f"{mpu_data['gx']:.1f}"
            gyro_y = f"{mpu_data['gy']:.1f}"
            gyro_z = f"{mpu_data['gz']:.1f}"
        else:
            acc_x = acc_y = acc_z = "N/A"
            gyro_x = gyro_y = gyro_z = "N/A"
        
        # Write to file
        with open(TELEMETRY_FILE, 'a') as f:
            f.write(f"{timestamp},{mode},{heading_str},{cardinal_str},{TARGET_HEADING},{error:.1f},{motor_left},{motor_right},{motor_auto},{gps_status},{lat},{lon},{alt},{sats},{acc_x},{acc_y},{acc_z},{gyro_x},{gyro_y},{gyro_z},{encoder_pos}\n")
        
    except Exception as e:
        print(f"Telemetry log error: {e}")

def print_telemetry_stats():
    """Print telemetry file statistics"""
    try:
        file_size = os.stat(TELEMETRY_FILE)[6]
        with open(TELEMETRY_FILE, 'r') as f:
            line_count = sum(1 for line in f) - 1  # Subtract header
        print(f"\n=== Telemetry Stats ===")
        print(f"File: {TELEMETRY_FILE}")
        print(f"Size: {file_size / 1024:.1f} KB ({file_size} bytes)")
        print(f"Data points: {line_count}")
        print(f"Max size: {MAX_FILE_SIZE / 1024 / 1024:.1f} MB")
        print("To download: Use Thonny's Files panel or run:")
        print("  with open('/telemetry.csv','r') as f: print(f.read())")
    except Exception as e:
        print(f"Stats error: {e}")

# ---------------- Startup ----------------
set_motor(LEFT_ESC, MIN_PULSE)
set_motor(RIGHT_ESC, MIN_PULSE)
set_motor(AUTO_MOTOR, MIN_PULSE)

# Initialize sensors
print("Initializing sensors...")
mpu_ok = init_mpu6050()
hmc_ok = init_hmc5883l()

# Initialize telemetry
telemetry_enabled = init_telemetry()

# Connect to camera
print("Connecting to ESP32-CAM...")
connect_to_camera()

time.sleep(2)

# ---------------- Encoder-Based Stepper Control (No Drift!) ----------------
STEP_PIN = 7
DIR_PIN  = 6
EN_PIN   = 5

step = Pin(STEP_PIN, Pin.OUT)
dir_pin  = Pin(DIR_PIN, Pin.OUT)
en   = Pin(EN_PIN, Pin.OUT)

# Encoder pins
ENC_CLK = Pin(10, Pin.IN, Pin.PULL_UP)
ENC_DT = Pin(11, Pin.IN, Pin.PULL_UP)

en.value(0)

# Encoder limits for panning range
ENCODER_MIN = -3
ENCODER_MAX = 3

# Stepper constants
step_delay_us = 1500  # Constant speed

# Encoder tracking
encoder_position = 0
last_clk_state = ENC_CLK.value()

# Direction tracking
direction = 1  # 1 = right, -1 = left
last_step_time = time.ticks_us()

def read_encoder():
    """Read encoder and update position"""
    global encoder_position, last_clk_state
    
    clk_state = ENC_CLK.value()
    dt_state = ENC_DT.value()
    
    if clk_state != last_clk_state:
        if dt_state != clk_state:
            encoder_position += 1
        else:
            encoder_position -= 1
        
        last_clk_state = clk_state

def update_stepper(mode):
    global direction, last_step_time

    # Only run stepper in autonomous and hover modes
    if mode == "manual":
        step.value(0)
        en.value(1)
        return

    # Enable motor
    en.value(0)

    # Read encoder continuously
    read_encoder()

    # Check if we've hit limits and need to reverse
    if encoder_position >= ENCODER_MAX:
        direction = -1  # Go left
    elif encoder_position <= ENCODER_MIN:
        direction = 1   # Go right

    # Make steps at constant speed
    now = time.ticks_us()
    if time.ticks_diff(now, last_step_time) >= step_delay_us * 2:
        last_step_time = now
        
        # Step motor
        dir_pin.value(1 if direction == 1 else 0)
        step.value(1)
        time.sleep_us(10)
        step.value(0)

# ---------------- Recording State ----------------
is_recording = False
last_record_state = False

def update_recording_led():
    """Blink LED when recording, solid when not"""
    global led_state, last_led_blink
    
    if is_recording:
        if time.ticks_diff(time.ticks_ms(), last_led_blink) > 250:
            led_state = not led_state
            led.value(led_state)
            last_led_blink = time.ticks_ms()
    else:
        led.value(0)

# ---------------- Sensor Reading Timing ----------------
last_sensor_read = time.ticks_ms()
SENSOR_READ_INTERVAL = 100
current_heading = 0

# ---------------- Main Loop ----------------
last_rx = time.ticks_ms()
FAILSAFE_MS = 300
mode = "unknown"

# Track motor values for telemetry
left = MIN_PULSE
right = MIN_PULSE
auto_motor_speed = MIN_PULSE

print("\n=== System Ready ===")
print(f"Target Heading: {TARGET_HEADING}°")
print("Channel 6: Mode selection")
print("  <= 1100: Autonomous (heading nav + stepper)")
print("  ~1500:   Hover (motors off + stepper)")
print("  >= 1900: Manual (RC control, stepper OFF)")
print("Channel 7: Recording control")
print(f"Telemetry: Logging at 1 sample/second to {TELEMETRY_FILE}")
print(f"\nStepper Mode: ENCODER-BASED (No Drift!)")
print(f"  - Pans until encoder reaches {ENCODER_MIN} or {ENCODER_MAX}")
print(f"  - Constant speed, reverses at encoder limits")
print(f"  - NO step counting = NO drift accumulation!")
print(f"  Repeats continuously in autonomous/hover modes\n")

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
        
        if current_record_state != last_record_state:
            if current_record_state:
                if send_camera_command("start_record"):
                    is_recording = True
            else:
                if send_camera_command("stop_record"):
                    is_recording = False
            
            last_record_state = current_record_state

        # Motor control
        throttle_raw = ch[2]
        steer_raw = ch[3]

        if mode == "manual":
            left, right = mix(throttle_raw, steer_raw)
            auto_motor_speed = MIN_PULSE

        elif mode == "autonomous":
            # Use heading-based navigation
            if hmc_ok and current_heading is not None:
                left, right = calculate_turn_motors(current_heading, TARGET_HEADING)
            else:
                left, right = MIN_PULSE, MIN_PULSE
            auto_motor_speed = FULL_SPEED

        else:
            # Hover mode - motors off
            left, right = MIN_PULSE, MIN_PULSE
            auto_motor_speed = MIN_PULSE

        set_motor(LEFT_ESC, left)
        set_motor(RIGHT_ESC, right)
        set_motor(AUTO_MOTOR, auto_motor_speed)

    # Read sensors periodically
    if time.ticks_diff(time.ticks_ms(), last_sensor_read) > SENSOR_READ_INTERVAL:
        last_sensor_read = time.ticks_ms()
        
        # Read IMU
        mpu_data = None
        if mpu_ok:
            mpu_data = read_mpu6050()
            if mpu_data:
                print(f"ACC: X={mpu_data['ax']:6.2f}g Y={mpu_data['ay']:6.2f}g Z={mpu_data['az']:6.2f}g | GYRO: X={mpu_data['gx']:7.1f}°/s Y={mpu_data['gy']:7.1f}°/s Z={mpu_data['gz']:7.1f}°/s")
        
        # Read Magnetometer with calibration and filtering
        cardinal = None
        if hmc_ok:
            current_heading = get_heading()
            if current_heading is not None:
                heading_error = angle_difference(current_heading, TARGET_HEADING)
                cardinal = heading_to_cardinal(current_heading)
                print(f"MAG: {current_heading:6.1f}° ({cardinal}) | Target: {TARGET_HEADING}° | Error: {heading_error:+6.1f}°")
        
        # Read GPS
        gps_data = parse_gps()
        if gps_data:
            if gps_data['status'] == 'FIX':
                print(f"GPS: [FIX] Lat={gps_data['lat']:.6f} Lon={gps_data['lon']:.6f} Alt={gps_data['alt']:.1f}m Sats={gps_data['sats']}")
            elif gps_data['status'] == 'NO_FIX':
                print(f"GPS: [NO FIX] Searching... Sats={gps_data['sats']}")
            elif gps_data['status'] == 'NO_DATA':
                print(f"GPS: [NO DATA] Not receiving GPS signal")
        
        # Print encoder status
        print(f"ENCODER: Position={encoder_position:+2d} | Range=[{ENCODER_MIN}, {ENCODER_MAX}] | Direction={'RIGHT' if direction == 1 else 'LEFT'}")
        
        # Log telemetry at 1Hz (1 sample per second)
        if telemetry_enabled and time.ticks_diff(time.ticks_ms(), last_log_time) >= LOG_INTERVAL_MS:
            last_log_time = time.ticks_ms()
            log_telemetry(mode, current_heading, cardinal, left, right, auto_motor_speed, mpu_data, gps_data, encoder_position)

    # Failsafe
    if time.ticks_diff(time.ticks_ms(), last_rx) > FAILSAFE_MS:
        set_motor(LEFT_ESC, MIN_PULSE)
        set_motor(RIGHT_ESC, MIN_PULSE)

    # Update peripherals
    update_stepper(mode)
    update_recording_led()

    time.sleep_ms(20)
