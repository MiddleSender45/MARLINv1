from machine import UART, Pin

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

while True:
    packet = read_ibus_packet()
    if packet:
        ch = parse_ibus_channels(packet)
        print(ch)
