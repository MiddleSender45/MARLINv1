from machine import Pin, I2C
import time

# --- I2C setup ---
i2c = I2C(1, scl=Pin(2), sda=Pin(1))  # ESP32 S3 pins

# --- PCA9685 registers ---
PCA9685_ADDR = 0x40
MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE

def write_byte(reg, value):
    i2c.writeto_mem(PCA9685_ADDR, reg, bytes([value]))

def set_pwm(channel, on, off):
    base = 0x06 + 4 * channel
    i2c.writeto_mem(PCA9685_ADDR, base, bytes([on & 0xFF]))
    i2c.writeto_mem(PCA9685_ADDR, base+1, bytes([on >> 8]))
    i2c.writeto_mem(PCA9685_ADDR, base+2, bytes([off & 0xFF]))
    i2c.writeto_mem(PCA9685_ADDR, base+3, bytes([off >> 8]))

# --- Initialize PCA9685 ---
write_byte(MODE1, 0x00)  # normal mode
write_byte(MODE2, 0x04)  # totem pole output

# --- Set PWM frequency to 50Hz ---
freq_hz = 50
prescale_val = int(round(25000000 / (4096 * freq_hz)) - 1)
oldmode = i2c.readfrom_mem(PCA9685_ADDR, MODE1, 1)[0]
newmode = (oldmode & 0x7F) | 0x10  # sleep
write_byte(MODE1, newmode)
write_byte(PRESCALE, prescale_val)
write_byte(MODE1, oldmode)
time.sleep(0.005)
write_byte(MODE1, oldmode | 0x80)  # restart

# --- ESC setup ---
ESC_CHANNELS = [1, 3]  # 3 ESCs connected
MIN_PULSE = 1000  # µs = 0% throttle
MID_PULSE = 1500  # µs = 50% throttle
MAX_PULSE = 2000  # µs = 100% throttle
RESOLUTION = 4096

def pulse_us_to_counts(pulse_us):
    period_us = 1000000 / freq_hz  # 50Hz → 20,000 µs
    counts = int((pulse_us / period_us) * RESOLUTION)
    return counts

def set_throttle(channel, pulse_us):
    counts = pulse_us_to_counts(pulse_us)
    set_pwm(channel, 0, counts)

def set_all_throttles(pulse_us):
    for ch in ESC_CHANNELS:
        set_throttle(ch, pulse_us)

# --- ESC calibration/arming sequence ---
print("ESC calibration sequence: max → min → 50%")
print("Power on ESCs now (motors disconnected for safety)")
time.sleep(2)

print("Sending max throttle (2ms) for 1 second")
set_all_throttles(MAX_PULSE)
time.sleep(1)

print("Sending min throttle (1ms) for 2 seconds")
set_all_throttles(MIN_PULSE)
time.sleep(2)

print("Sending 50% throttle (1.5ms) for 2 seconds")
set_all_throttles(MID_PULSE)
time.sleep(2)

print("Returning to 0% throttle")
set_all_throttles(MIN_PULSE)
time.sleep(1)

print("ESC test done! You can now connect the motors safely.")

