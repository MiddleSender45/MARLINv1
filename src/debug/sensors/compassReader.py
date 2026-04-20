from machine import Pin, I2C
import math
import time

# ---------- I2C ----------
i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=100000)

# ---------- HMC5883L ----------
HMC_ADDR = 0x1E

i2c.writeto_mem(HMC_ADDR, 0x00, b'\x70')
i2c.writeto_mem(HMC_ADDR, 0x01, b'\xA0')
i2c.writeto_mem(HMC_ADDR, 0x02, b'\x00')

def twos_comp16(val):
    if val >= 32768:
        return val - 65536
    return val

def read_mag():
    data = i2c.readfrom_mem(HMC_ADDR, 0x03, 6)

    x = twos_comp16(int.from_bytes(data[0:2], 'big'))
    z = twos_comp16(int.from_bytes(data[2:4], 'big'))
    y = twos_comp16(int.from_bytes(data[4:6], 'big'))

    return x, y, z


# ---------- MPU6050 ----------
MPU_ADDR = 0x68

# wake it up
i2c.writeto_mem(MPU_ADDR, 0x6B, b'\x00')

def read_mpu():
    data = i2c.readfrom_mem(MPU_ADDR, 0x3B, 14)

    ax = twos_comp16(int.from_bytes(data[0:2], 'big'))
    ay = twos_comp16(int.from_bytes(data[2:4], 'big'))
    az = twos_comp16(int.from_bytes(data[4:6], 'big'))

    gx = twos_comp16(int.from_bytes(data[8:10], 'big'))
    gy = twos_comp16(int.from_bytes(data[10:12], 'big'))
    gz = twos_comp16(int.from_bytes(data[12:14], 'big'))

    # convert to usable units
    ax /= 16384
    ay /= 16384
    az /= 16384

    gx /= 131
    gy /= 131
    gz /= 131

    return ax, ay, az, gx, gy, gz


# ---------- CALIBRATION ----------
OFFSET = [-128.5, 149.0, 120.0]

# ---------- FILTER ----------
heading_filtered = 0
ALPHA = 0.15

def get_heading():
    global heading_filtered

    mx, my, mz = read_mag()

    mx -= OFFSET[0]
    my -= OFFSET[1]

    heading = math.degrees(math.atan2(my, mx))

    if heading < 0:
        heading += 360

    diff = heading - heading_filtered

    if diff > 180:
        diff -= 360
    elif diff < -180:
        diff += 360

    heading_filtered += ALPHA * diff
    heading_filtered %= 360

    return heading_filtered

def heading_to_cardinal(h):
    dirs = ["N","NE","E","SE","S","SW","W","NW"]
    return dirs[int((h + 22.5)//45) % 8]


# ---------- LOOP ----------
while True:

    heading = get_heading()
    cardinal = heading_to_cardinal(heading)

    ax, ay, az, gx, gy, gz = read_mpu()

    print(f"{heading:.1f}° ({cardinal}) | "
          f"A: {ax:.2f},{ay:.2f},{az:.2f}g | "
          f"G: {gx:.1f},{gy:.1f},{gz:.1f}°/s")

    time.sleep(0.2)

