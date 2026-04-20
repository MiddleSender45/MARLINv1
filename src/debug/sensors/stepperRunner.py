from machine import Pin
import time

# Pin definitions
STEP_PIN = 7
DIR_PIN  = 6
EN_PIN   = 5

# Setup pins
step = Pin(STEP_PIN, Pin.OUT)
dir  = Pin(DIR_PIN, Pin.OUT)
en   = Pin(EN_PIN, Pin.OUT)

# Enable driver (most drivers: LOW = enabled)
en.value(0)

def step_motor(steps, direction=1, delay_us=800):
    """
    steps: number of steps to move
    direction: 1 = CW, 0 = CCW
    delay_us: controls speed (lower = faster)
    """
    dir.value(direction)
    
    for _ in range(steps):
        step.value(1)
        time.sleep_us(delay_us)
        step.value(0)
        time.sleep_us(delay_us)

# ------------------------
# Example usage
# ------------------------

while True:
    print("Forward")
    step_motor(200, direction=1)   # 1 full revolution on 1.8° motor
    time.sleep(1)

    print("Backward")
    step_motor(200, direction=0)
    time.sleep(1)

