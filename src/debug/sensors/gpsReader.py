from machine import UART, Pin
import time

# GPS Setup
gps_uart = UART(1, baudrate=9600, tx=Pin(17), rx=Pin(18))

print("=== GPS UART Test ===")
print("Listening for GPS data on UART1 (TX=17, RX=18)...")
print("If working, you should see NMEA sentences starting with $GP or $GN")
print("Press Ctrl+C to stop\n")

while True:
    if gps_uart.any():
        data = gps_uart.read()
        if data:
            try:
                print(data.decode('utf-8', 'ignore'), end='')
            except:
                print(data)  # Print raw bytes if decode fails
    time.sleep(0.01)
