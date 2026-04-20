from machine import Pin, I2C
import time

SDA_PIN = 8
SCL_PIN = 9

i2c = I2C(0, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=100000)

HMC_ADDR = 0x1E

# Init HMC5883L
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


mag_min = [9999, 9999, 9999]
mag_max = [-9999, -9999, -9999]

print("Rotate robot slowly in ALL directions for 25 seconds...")
time.sleep(2)

start = time.time()

while time.time() - start < 25:

    mx, my, mz = read_mag()

    mag_min[0] = min(mag_min[0], mx)
    mag_min[1] = min(mag_min[1], my)
    mag_min[2] = min(mag_min[2], mz)

    mag_max[0] = max(mag_max[0], mx)
    mag_max[1] = max(mag_max[1], my)
    mag_max[2] = max(mag_max[2], mz)

    time.sleep(0.05)

offset = [(mag_max[i] + mag_min[i]) / 2 for i in range(3)]

print("\nCALIBRATION COMPLETE")
print("Paste this into your main code:\n")
print("OFFSET =", offset)

