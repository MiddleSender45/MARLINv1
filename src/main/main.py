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

# Try to initialize WiFi, but don't crash if it fails
try:
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wifi_available = True
except Exception as e:
    print(f"WiFi initialization failed: {e}")
    wlan = None
    wifi_available = False

def connect_to_camera():
    """Connect to ESP32-CAM access point - non-blocking"""
    global wifi_available
    
    if not wifi_available or wlan is None:
        return False
    
    try:
        if not wlan.isconnected():
            print(f"Attempting to connect to {CAMERA_SSID}...")
            wlan.connect(CAMERA_SSID, CAMERA_PASSWORD)
            
            timeout = 5  # Reduced timeout
            while not wlan.isconnected() and timeout > 0:
                print(".", end="")
                time.sleep(1)
                timeout -= 1
            
            if wlan.isconnected():
                print(f"\nCamera connected! IP: {wlan.ifconfig()[0]}")
                return True
            else:
                print("\nCamera connection failed - continuing without camera")
                return False
        return True
    except Exception as e:
        print(f"\nCamera connection error: {e} - continuing without camera")
        wifi_available = False
        return False

def send_camera_command(command):
    """Send command to camera (start_record or stop_record) - non-blocking"""
    global wifi_available
    
    if not wifi_available or wlan is None:
        return False
    
    try:
        if not wlan.isconnected():
            return False
            
        url = f"http://{CAMERA_IP}/command"
        response = urequests.post(url, data=command)
        result = response.text
        response.close()
        print(f"Camera response: {result}")
        return True
    except Exception as e:
        # Silently fail - don't spam console
        return False

# ---------------- LED Indicator ----------------
LED_PIN = 48
led = Pin(LED_PIN, Pin.OUT)
led.value(0)

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
LEFT_ESC = 1  # SWAPPED - was 3
RIGHT_ESC = 3  # SWAPPED - was 1
MIN_PULSE = 1000
MAX_PULSE = 2000
AUTO_MOTOR = 4
FULL_SPEED = MAX_PULSE

# Motor trim controlled by Channel 9 (dynamically adjustable)
# Channel 9: 1000 = -500 trim (strong right bias)
#           1500 = 0 trim (no bias)
#           2000 = +500 trim (strong left bias)

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

# ---------------- Path Planning System ----------------
PATH_FOLDER = '/path_logs'
PATH_RECORD_INTERVAL_MS = 100  # Record path every 100ms (10 Hz)
current_path = []  # Stores waypoint count during recording
path_playback = []  # Buffer for loaded waypoints (small rolling buffer)
playback_index = 0
last_path_record = time.ticks_ms()
CURRENT_PATH_FILE = None
path_completed = False

# Streaming playback variables
PLAYBACK_PATH_FILE = None  # Path file being played back
playback_file_handle = None  # Open file handle during playback
path_total_waypoints = 0  # Total waypoints in file
path_start_time = 0  # First waypoint timestamp
path_end_time = 0  # Last waypoint timestamp
PLAYBACK_BUFFER_SIZE = 50  # Keep only 50 waypoints in memory at once

def get_next_path_number():
    """Get the next sequential path number"""
    try:
        # Create folder if it doesn't exist
        try:
            os.mkdir(PATH_FOLDER)
            print(f"Created path folder: {PATH_FOLDER}")
        except:
            pass
        
        # List all files in the folder
        try:
            files = os.listdir(PATH_FOLDER)
        except:
            return 1
        
        # Find highest number
        max_num = 0
        for filename in files:
            if filename.startswith('path_') and filename.endswith('.csv'):
                try:
                    num_str = filename[5:9]  # Get the 4-digit number
                    num = int(num_str)
                    if num > max_num:
                        max_num = num
                except:
                    pass
        
        return max_num + 1
    except Exception as e:
        print(f"Error getting path number: {e}")
        return 1

def start_path_recording():
    """Start recording a new path - opens file immediately"""
    global current_path, CURRENT_PATH_FILE, last_path_record
    
    current_path = []  # Keep for waypoint count display only
    last_path_record = time.ticks_ms()
    
    # Get next path number
    path_number = get_next_path_number()
    filename = f"path_{path_number:04d}.csv"
    CURRENT_PATH_FILE = f"{PATH_FOLDER}/{filename}"
    
    # Open file and write header immediately
    try:
        with open(CURRENT_PATH_FILE, 'w') as f:
            f.write("time_ms,left_motor,right_motor\n")
        print(f"\n*** STARTED PATH RECORDING #{path_number} ***")
        print(f"File: {CURRENT_PATH_FILE}")
        print("Drive the boat manually - path will be recorded\n")
        print("NOTE: Waypoints written to file in real-time (no memory limit!)\n")
    except Exception as e:
        print(f"Error creating path file: {e}")
        CURRENT_PATH_FILE = None

def record_path_waypoint(left_motor, right_motor):
    """Record a waypoint to the current path - writes directly to file"""
    global current_path, last_path_record, CURRENT_PATH_FILE
    
    if CURRENT_PATH_FILE is None:
        return
    
    now = time.ticks_ms()
    if time.ticks_diff(now, last_path_record) < PATH_RECORD_INTERVAL_MS:
        return
    
    last_path_record = now
    
    # Write waypoint directly to file (no RAM storage - unlimited recording!)
    try:
        with open(CURRENT_PATH_FILE, 'a') as f:
            f.write(f"{time.ticks_ms()},{left_motor},{right_motor}\n")
        
        # Keep count in memory for display purposes only
        current_path.append(None)  # Just increment count, don't store data
    except Exception as e:
        print(f"Error writing waypoint: {e}")

def save_path():
    """Save the current path to file - file is already written, just confirm completion"""
    global current_path, CURRENT_PATH_FILE
    
    if CURRENT_PATH_FILE is None:
        print("\n*** NO PATH FILE (recording not started properly) ***\n")
        return False
    
    if len(current_path) == 0:
        print("\n*** NO PATH TO SAVE (path was empty) ***\n")
        return False
    
    try:
        # File is already written in real-time - just need to calculate duration
        # Read first and last lines to get duration
        with open(CURRENT_PATH_FILE, 'r') as f:
            lines = f.readlines()
            if len(lines) > 1:  # Header + at least one waypoint
                first_data = lines[1].strip().split(',')
                last_data = lines[-1].strip().split(',')
                
                first_time = int(first_data[0])
                last_time = int(last_data[0])
                duration = (last_time - first_time) / 1000.0
                
                print(f"\n*** PATH SAVED ***")
                print(f"File: {CURRENT_PATH_FILE}")
                print(f"Waypoints: {len(current_path)}")
                print(f"Duration: {duration:.1f} seconds")
                print(f"File written in real-time - no memory limits!\n")
            else:
                print("\n*** PATH SAVED (but was empty) ***\n")
        
        return True
    except Exception as e:
        print(f"Error confirming path save: {e}")
        return False

def load_most_recent_path():
    """Load the most recent path file for playback - streaming mode (low memory)"""
    global path_playback, playback_index, path_completed
    global PLAYBACK_PATH_FILE, path_total_waypoints, path_start_time, path_end_time
    
    try:
        files = os.listdir(PATH_FOLDER)
    except:
        print("No paths available to load")
        return False
    
    # Find highest numbered path
    max_num = 0
    latest_file = None
    for filename in files:
        if filename.startswith('path_') and filename.endswith('.csv'):
            try:
                num_str = filename[5:9]
                num = int(num_str)
                if num > max_num:
                    max_num = num
                    latest_file = filename
            except:
                pass
    
    if latest_file is None:
        print("No valid path files found")
        return False
    
    # Load the path metadata only (not all waypoints!)
    PLAYBACK_PATH_FILE = f"{PATH_FOLDER}/{latest_file}"
    try:
        # Read file to get metadata
        with open(PLAYBACK_PATH_FILE, 'r') as f:
            lines = f.readlines()
            
            if len(lines) < 2:
                print("Path file is empty")
                return False
            
            # Skip header
            # Get first waypoint
            first_parts = lines[1].strip().split(',')
            path_start_time = int(first_parts[0])
            
            # Get last waypoint
            last_parts = lines[-1].strip().split(',')
            path_end_time = int(last_parts[0])
            
            # Count waypoints (excluding header)
            path_total_waypoints = len(lines) - 1
        
        playback_index = 0
        path_completed = False
        path_playback = []  # Will be filled during playback
        
        duration = (path_end_time - path_start_time) / 1000.0
        
        print(f"\n*** LOADED PATH FOR AUTONOMOUS MODE (STREAMING) ***")
        print(f"File: {PLAYBACK_PATH_FILE}")
        print(f"Waypoints: {path_total_waypoints}")
        print(f"Duration: {duration:.1f} seconds")
        print(f"Memory mode: Streaming (only {PLAYBACK_BUFFER_SIZE} waypoints in RAM at once)\n")
        
        return True
    except Exception as e:
        print(f"Error loading path: {e}")
        return False

def execute_path(playback_start_time):
    """Execute the loaded path with streaming (on-demand file reading) - returns (left_motor, right_motor)"""
    global playback_index, path_completed, path_playback
    global PLAYBACK_PATH_FILE, path_total_waypoints, path_start_time, path_end_time
    
    if PLAYBACK_PATH_FILE is None or path_total_waypoints == 0:
        path_completed = True
        return (MIN_PULSE, MIN_PULSE)
    
    # Calculate elapsed time since playback started
    elapsed_ms = time.ticks_diff(time.ticks_ms(), playback_start_time)
    
    # Get path duration
    path_duration = path_end_time - path_start_time
    
    # Stop at end of path and mark as completed
    if elapsed_ms > path_duration:
        if not path_completed:
            path_completed = True
            print("*** PATH EXECUTION COMPLETE - STOPPED ***")
        return (MIN_PULSE, MIN_PULSE)
    
    # Load waypoints from file on-demand (streaming)
    try:
        # Calculate which waypoint we should be at
        target_waypoint = int((elapsed_ms / path_duration) * path_total_waypoints)
        target_waypoint = max(0, min(target_waypoint, path_total_waypoints - 1))
        
        # Read the current and next waypoint from file
        with open(PLAYBACK_PATH_FILE, 'r') as f:
            # Skip header
            f.readline()
            
            # Skip to target waypoint
            for i in range(target_waypoint):
                f.readline()
            
            # Read current waypoint
            current_line = f.readline().strip()
            if not current_line:
                path_completed = True
                return (MIN_PULSE, MIN_PULSE)
            
            current_parts = current_line.split(',')
            current_wp = {
                'time': int(current_parts[0]),
                'left': int(current_parts[1]),
                'right': int(current_parts[2])
            }
            
            # Read next waypoint for interpolation
            next_line = f.readline().strip()
            if next_line:
                next_parts = next_line.split(',')
                next_wp = {
                    'time': int(next_parts[0]),
                    'left': int(next_parts[1]),
                    'right': int(next_parts[2])
                }
                
                # Interpolate between current and next waypoint
                wp_current_time = current_wp['time'] - path_start_time
                wp_next_time = next_wp['time'] - path_start_time
                
                if wp_next_time > wp_current_time:
                    t = (elapsed_ms - wp_current_time) / (wp_next_time - wp_current_time)
                    t = max(0.0, min(1.0, t))
                    
                    left = int(current_wp['left'] * (1 - t) + next_wp['left'] * t)
                    right = int(current_wp['right'] * (1 - t) + next_wp['right'] * t)
                else:
                    left = current_wp['left']
                    right = current_wp['right']
            else:
                # Last waypoint
                left = current_wp['left']
                right = current_wp['right']
            
            playback_index = target_waypoint
            return (left, right)
            
    except Exception as e:
        print(f"Path execution error: {e}")
        path_completed = True
        return (MIN_PULSE, MIN_PULSE)

# ---------------- Telemetry Logging Setup ----------------
TELEMETRY_FOLDER = '/telemetry_logs'
LOG_INTERVAL_MS = 1000  # 1 sample per second
MAX_FILE_SIZE = 2 * 1024 * 1024  # 2 MB max file size
last_log_time = time.ticks_ms()
TELEMETRY_FILE = None

def get_next_log_number():
    """Get the next sequential log number by reading existing files"""
    try:
        # Create folder if it doesn't exist
        try:
            os.mkdir(TELEMETRY_FOLDER)
            print(f"Created telemetry folder: {TELEMETRY_FOLDER}")
        except:
            pass  # Folder already exists
        
        # List all files in the folder
        try:
            files = os.listdir(TELEMETRY_FOLDER)
        except:
            return 1  # No files, start at 1
        
        # Find highest number
        max_num = 0
        for filename in files:
            if filename.startswith('log_') and filename.endswith('.csv'):
                try:
                    # Extract number from filename like "log_0042.csv"
                    num_str = filename[4:8]  # Get the 4-digit number
                    num = int(num_str)
                    if num > max_num:
                        max_num = num
                except:
                    pass
        
        return max_num + 1
    except Exception as e:
        print(f"Error getting log number: {e}")
        return 1

def init_telemetry():
    """Initialize telemetry file with headers - creates new numbered file each session"""
    global TELEMETRY_FILE
    
    try:
        # Get next log number
        log_number = get_next_log_number()
        
        # Create filename with zero-padded number (e.g., log_0001.csv, log_0042.csv)
        filename = f"log_{log_number:04d}.csv"
        TELEMETRY_FILE = f"{TELEMETRY_FOLDER}/{filename}"
        
        # Create new file with headers
        with open(TELEMETRY_FILE, 'w') as f:
            f.write("time_ms,mode,path_planning,heading,cardinal,target,error,left_motor,right_motor,auto_motor,gps_status,lat,lon,alt,sats,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,encoder_pos\n")
        
        print(f"Created telemetry log #{log_number}: {TELEMETRY_FILE}")
        return True
    except Exception as e:
        print(f"Telemetry init error: {e}")
        return False
    
def log_telemetry(mode, path_planning, heading_val, cardinal_val, motor_left, motor_right, motor_auto, mpu_data, gps_data, encoder_pos):
    """Log one line of telemetry data"""
    global TELEMETRY_FILE
    
    if TELEMETRY_FILE is None:
        return
    
    try:
        # Check file size before writing
        try:
            file_size = os.stat(TELEMETRY_FILE)[6]
            if file_size > MAX_FILE_SIZE:
                print(f"Max file size reached for {TELEMETRY_FILE}")
                print("Starting new log file...")
                init_telemetry()  # Create new numbered file
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
            f.write(f"{timestamp},{mode},{path_planning},{heading_str},{cardinal_str},{TARGET_HEADING},{error:.1f},{motor_left},{motor_right},{motor_auto},{gps_status},{lat},{lon},{alt},{sats},{acc_x},{acc_y},{acc_z},{gyro_x},{gyro_y},{gyro_z},{encoder_pos}\n")
        
    except Exception as e:
        print(f"Telemetry log error: {e}")

def print_telemetry_stats():
    """Print telemetry file statistics"""
    global TELEMETRY_FILE
    
    if TELEMETRY_FILE is None:
        print("No telemetry file active")
        return
    
    try:
        file_size = os.stat(TELEMETRY_FILE)[6]
        with open(TELEMETRY_FILE, 'r') as f:
            line_count = sum(1 for line in f) - 1  # Subtract header
        print(f"\n=== Telemetry Stats ===")
        print(f"File: {TELEMETRY_FILE}")
        print(f"Size: {file_size / 1024:.1f} KB ({file_size} bytes)")
        print(f"Data points: {line_count}")
        print(f"Max size: {MAX_FILE_SIZE / 1024 / 1024:.1f} MB")
        print(f"\nAll logs stored in: {TELEMETRY_FOLDER}")
        print("To list all logs: os.listdir('/telemetry_logs')")
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

# Try to connect to camera (non-blocking, continues if fails)
print("Attempting camera connection...")
camera_connected = connect_to_camera()
if not camera_connected:
    print("Running WITHOUT camera module")

time.sleep(1)  # Reduced delay

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
ENCODER_MIN = -4
ENCODER_MAX = 4

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
    """Update stepper motor based on mode and encoder position"""
    global direction, last_step_time

    # Only run stepper in autonomous and hover modes
    if mode == "manual":
        step.value(0)
        en.value(1)  # Disable stepper
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
FAILSAFE_MS = 1500
mode = "unknown"
path_planning_mode = False
last_path_planning_state = False
playback_start_time = 0

# Debounce timing for mode switches (1 second hold required)
DEBOUNCE_MS = 1000
path_planning_debounce_start = 0
path_planning_pending_state = False
mode_debounce_start = 0
mode_pending = "unknown"

# Track motor values for telemetry
left = MIN_PULSE
right = MIN_PULSE
auto_motor_speed = MIN_PULSE

print("\n=== System Ready ===")
print(f"Target Heading: {TARGET_HEADING}°")
print("Channel 4: Path Planning switch")
print("  >= 1900: START recording new path (hold 1 sec)")
print("  <= 1100: SAVE recorded path (hold 1 sec)")
print("Channel 6: Mode selection (hold 1 sec to switch)")
print("  <= 1100: Autonomous (executes most recent path)")
print("  ~1500:   Hover (motors off + stepper)")
print("  >= 1900: Manual (RC control, stepper OFF)")
print("Channel 7: Recording control")
print("Channel 9: Motor Trim (real-time adjustment)")
print("  1000 = -500 trim (strong right bias)")
print("  1500 = 0 trim (neutral)")
print("  2000 = +500 trim (strong left bias)")
print(f"Telemetry: Logging to {TELEMETRY_FILE}")
print(f"Path logs: Stored in {PATH_FOLDER}")
print(f"\nStepper Mode: ENCODER-BASED (No Drift!)")
print(f"  - Pans until encoder reaches {ENCODER_MIN} or {ENCODER_MAX}")
print(f"  - Repeats continuously in autonomous/hover modes")
print(f"\nMode switching: 1-second hold required (prevents glitches)")
print(f"Autonomous mode: Path execution stops when complete\n")

# Try to load most recent path on startup
load_most_recent_path()

while True:
    packet = read_ibus_packet()
    if packet:
        ch = parse_ibus_channels(packet)
        last_rx = time.ticks_ms()

        # Path planning mode from channel 4 - WITH DEBOUNCING
        path_ch = ch[4]
        current_path_planning_state = path_ch >= 1900
        
        # Check if state is different from current mode
        if current_path_planning_state != path_planning_mode:
            # State is different - check if we need to start debounce timer
            if current_path_planning_state != path_planning_pending_state:
                # New state detected - start debounce timer
                path_planning_pending_state = current_path_planning_state
                path_planning_debounce_start = time.ticks_ms()
            else:
                # Same pending state - check if enough time has passed
                if time.ticks_diff(time.ticks_ms(), path_planning_debounce_start) >= DEBOUNCE_MS:
                    # Held for 1 second - execute the change
                    if current_path_planning_state:
                        # Start path recording
                        start_path_recording()
                        path_planning_mode = True
                    else:
                        # Save path and load it for autonomous mode
                        if path_planning_mode:
                            save_path()
                            load_most_recent_path()
                        path_planning_mode = False
        else:
            # State matches current mode - reset debounce
            path_planning_pending_state = current_path_planning_state
            path_planning_debounce_start = time.ticks_ms()

        # Mode from channel 6 - WITH DEBOUNCING
        mode_ch = ch[6]
        prev_mode = mode
        
        # Determine what mode the RC switch is requesting
        if mode_ch <= 1100:
            requested_mode = "autonomous"
        elif 1400 <= mode_ch <= 1600:
            requested_mode = "hover"
        elif mode_ch >= 1900:
            requested_mode = "manual"
        else:
            requested_mode = "unknown"
        
        # Check if requested mode is different from current mode
        if requested_mode != mode:
            # Mode is different - check if we need to start debounce timer
            if requested_mode != mode_pending:
                # New mode detected - start debounce timer
                mode_pending = requested_mode
                mode_debounce_start = time.ticks_ms()
            else:
                # Same pending mode - check if enough time has passed
                if time.ticks_diff(time.ticks_ms(), mode_debounce_start) >= DEBOUNCE_MS:
                    # Held for 1 second - execute the mode change
                    prev_mode = mode
                    mode = requested_mode
                    
                    # Reset playback when entering autonomous mode
                    if mode == "autonomous" and prev_mode != "autonomous":
                        playback_start_time = time.ticks_ms()
                        playback_index = 0
                        path_completed = False
                        print(f"\n=== AUTONOMOUS MODE STARTED - Beginning path playback ===\n")
                    elif mode != "autonomous":
                        playback_start_time = 0
                        if prev_mode == "autonomous":
                            path_completed = False
        else:
            # Mode matches current - reset debounce
            mode_pending = requested_mode
            mode_debounce_start = time.ticks_ms()

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
        
        # Calculate motor trim from channel 9
        # ch[9]: 1000->2000 maps to trim: -500->+500
        trim_ch = ch[9]
        motor_trim = int((trim_ch - 1500) * (500.0 / 500.0))  # Center at 1500, scale to ±500

        if mode == "manual" or path_planning_mode:
            # Manual control or path planning (same thing - user drives)
            left, right = mix(throttle_raw, steer_raw)
            auto_motor_speed = MIN_PULSE
            
            # Record waypoints if in path planning mode
            if path_planning_mode:
                record_path_waypoint(left, right)

        elif mode == "autonomous":
            # Execute recorded path
            if path_total_waypoints > 0:
                left, right = execute_path(playback_start_time)
            else:
                # No path loaded - stop
                left, right = MIN_PULSE, MIN_PULSE
            
            auto_motor_speed = FULL_SPEED

        else:
            # Hover mode - motors off
            left, right = MIN_PULSE, MIN_PULSE
            auto_motor_speed = MIN_PULSE

        # Apply motor trim to compensate for skew
        left_trimmed = left + motor_trim
        right_trimmed = right - motor_trim
        
        # Clamp to valid range
        left_trimmed = max(MIN_PULSE, min(MAX_PULSE, left_trimmed))
        right_trimmed = max(MIN_PULSE, min(MAX_PULSE, right_trimmed))
        
        set_motor(LEFT_ESC, left_trimmed)
        set_motor(RIGHT_ESC, right_trimmed)
        set_motor(AUTO_MOTOR, auto_motor_speed)
        
        # Debug: Print motor values every loop (comment out after testing)
        # print(f"TRIM={motor_trim} | MOTORS: L={left}->{left_trimmed} R={right}->{right_trimmed}")

    # Failsafe
    if time.ticks_diff(time.ticks_ms(), last_rx) > FAILSAFE_MS:
        set_motor(LEFT_ESC, MIN_PULSE)
        set_motor(RIGHT_ESC, MIN_PULSE)
        set_motor(AUTO_MOTOR, MIN_PULSE)

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
        
        # Print status
        mode_str = "PATH_PLAN" if path_planning_mode else mode.upper()
        if path_planning_mode:
            print(f"MODE: {mode_str} | Recording waypoints: {len(current_path)}")
        elif mode == "autonomous":
            if path_total_waypoints > 0:
                completion_pct = (playback_index / path_total_waypoints) * 100 if path_total_waypoints > 0 else 0
                status = "COMPLETE - STOPPED" if path_completed else "EXECUTING"
                print(f"MODE: {mode_str} | {status} | Progress: {playback_index}/{path_total_waypoints} ({completion_pct:.1f}%)")
            else:
                print(f"MODE: {mode_str} | NO PATH LOADED")
        else:
            print(f"MODE: {mode_str}")
        
        print(f"ENCODER: Position={encoder_position:+2d} | Range=[{ENCODER_MIN}, {ENCODER_MAX}] | Direction={'RIGHT' if direction == 1 else 'LEFT'}")
        
        # Log telemetry at 1Hz (1 sample per second)
        if telemetry_enabled and time.ticks_diff(time.ticks_ms(), last_log_time) >= LOG_INTERVAL_MS:
            last_log_time = time.ticks_ms()
            log_telemetry(mode, path_planning_mode, current_heading, cardinal, left, right, auto_motor_speed, mpu_data, gps_data, encoder_position)

    # Update peripherals
    update_stepper(mode)
    update_recording_led()

    time.sleep_ms(20)
