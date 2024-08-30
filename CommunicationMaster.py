from pyfirmata2 import Arduino, util
import time

COM_PORT = '/dev/umrt-arm'
SYSEX_COMMAND_ECHO = 0x00
SYSEX_COMMAND_SET_SPEED = 0x01
SYSEX_COMMAND_SEND_STEP = 0x03

MOTOR_IDS = [0, 1]

def pack_32(integer):
    # Little-endian
    #
    # e.g. for 0xDEAD_BEEF:
    # 1101 1110 1010 1101 1011 1110 1110 1111
    # 3333 3333 2222 2222 1111 1111 0000 0000
    # packed = [ 0xEF, 0xBE, 0xAD, 0xDE ]
    return bytearray([
    integer & 0xFF,       # bits [7, 0]
    integer >> 8 & 0xFF,  # bits [15, 8]
    integer >> 16 & 0xFF, # bits [23, 16]
    integer >> 24 & 0xFF  # bits [31, 24]
    ])

def pack_16(integer):
    # Little-endian
    #
    # e.g. for 0xBEEF:
    # 1011 1110 1110 1111
    # 1111 1111 0000 0000
    # packed = [ 0xEF, 0xBE ]
    return bytearray([
    integer & 0xFF,     # bits [7, 0]
    integer >> 8 & 0xFF # bits [15, 8]
    ])

def firmatify(pack):
    # Convert a packed bytearray to the 7-bit packets Firmata uses.
    # Must be called on the pack provided to send_sysex
    # Useful for checking a == decode_32(firmatify(pack_32(a)))
    #
    # e.g.  for [0xEF, 0xBE, 0xAD, 0xDE]:
    # [1101 1110, 1010 1101, 1011 1110, 1110 1111]
    # firmatafied = [ 0101 1110, 0000 0001, 0010 1101, 0000 0001, 0011 1110, 0000 0001, 0110 1111, 0000 0001 ]
    #             = [ 0x5E, 0x01, 0x2D, 0x01, 0x3E, 0x01, 0x6F, 0x01]
    b = bytearray()
    for p in pack:
        b.append(p & 0x7F)
        b.append((p & 0x80) >> 7)
    return b

def decode_32(data):
    # Decode Firmata 7-bit packets into a 32-bit integer
    # See firmatify for an explanation of what Firmata does to packets
    return data[0] | data[1] << 7 \
        | (data[2] | data[3] << 7) << 8 \
        | (data[4] | data[5] << 7) << 16 \
        | (data[6] | data[7] << 7) << 24

def decode_16(data):
    # Decode Firmata 7-bit packets into a 16-bit integer
    # See firmatify for an explanation of what Firmata does to packets
    return data[0] | data[1] << 7 \
        | (data[2] | data[3] << 7) << 8

def on_echo_text(*data):
    print(util.two_byte_iter_to_str(data))

def on_echo_int32(*data):
    print(decode_32(data))

def on_echo_int16(*data):
    print(decode_16(data))

def on_echo_raw(*data):
    print(data)

# Setup Firmata
b = Arduino(COM_PORT)
b.add_cmd_handler(0x71, on_echo_text)
it = util.Iterator(b)
it.start()

# Send text echo
b.add_cmd_handler(SYSEX_COMMAND_ECHO, on_echo_text)
b.send_sysex(SYSEX_COMMAND_ECHO, util.str_to_two_byte_iter("hello world"))

time.sleep(1)

# Send some numerical echos
b.add_cmd_handler(SYSEX_COMMAND_ECHO, on_echo_int32)
b.send_sysex(SYSEX_COMMAND_ECHO, firmatify(pack_32(0xDEAD_BEEF)))
b.send_sysex(SYSEX_COMMAND_ECHO, firmatify(pack_32(1000)))
b.send_sysex(SYSEX_COMMAND_ECHO, firmatify(pack_32(32767)))

time.sleep(1)

# Send the numerical echos raw
b.add_cmd_handler(SYSEX_COMMAND_ECHO, on_echo_raw)
b.send_sysex(SYSEX_COMMAND_ECHO, firmatify(pack_32(0xDEAD_BEEF)))
b.send_sysex(SYSEX_COMMAND_ECHO, firmatify(pack_32(1000)))
b.send_sysex(SYSEX_COMMAND_ECHO, firmatify(pack_32(32767)))

time.sleep(1)

for motor in MOTOR_IDS:
    # Send speed of 2 RPM for 5 seconds, then 1 RPM in other direction for 5 seconds, then stop
    b.send_sysex(SYSEX_COMMAND_SET_SPEED, firmatify(bytearray([motor]) + pack_16(20)))
    time.sleep(5)
    b.send_sysex(SYSEX_COMMAND_SET_SPEED,  firmatify(bytearray([motor]) + pack_16(-10)))
    time.sleep(5)
    b.send_sysex(SYSEX_COMMAND_SET_SPEED, firmatify(bytearray([motor]) + pack_16(0)))
    
    # Step forward 20 steps at 10 RPM, then back 10 steps at 5 RPM
    b.send_sysex(SYSEX_COMMAND_SEND_STEP, firmatify(bytearray([motor]) + pack_16(20) + pack_16(100)))
    time.sleep(1)
    b.send_sysex(SYSEX_COMMAND_SEND_STEP, firmatify(bytearray([motor]) + pack_16(10) + pack_16(-50)))
    
    time.sleep(1)

