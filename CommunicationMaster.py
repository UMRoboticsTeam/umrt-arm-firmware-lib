from pyfirmata2 import Arduino, util
import time

COM_PORT = '/dev/umrt-arm'
SYSEX_COMMAND_ECHO = 0x00
SYSEX_COMMAND_SET_SPEED = 0x01
SYSEX_COMMAND_GET_SPEED = 0x02
SYSEX_COMMAND_SEND_STEP = 0x03
SYSEX_COMMAND_SEEK_POS = 0x04
SYSEX_COMMAND_GET_POS = 0x05
SYSEX_COMMAND_SET_GRIPPER = 0x06

MOTOR_IDS = [2]

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

def defirmatify(data):
    # Decode Firmata 7-bit packets into a bytearray of 8-bit packets
    # See firmatify for an explanation of what Firmata does to packets
    b = bytearray()
    for i in range(0, len(data), 2):
        b.append(data[i] | data[i + 1] << 7)
    return b

def decode_32(data, offset=0, signed=True):
    return int.from_bytes(data[offset:offset+4], byteorder='little', signed=signed)

def decode_16(data, offset=0, signed=True):
    return int.from_bytes(data[offset:offset+2], byteorder='little', signed=signed)

def decode_8(data, offset=0, signed=True):
    return int.from_bytes(data[offset:offset+1], byteorder='little', signed=signed)

def on_echo_text(*data):
    print(util.two_byte_iter_to_str(data))

def on_echo_int32(*data):
    data = defirmatify(data)
    print(decode_32(data, 0))

def on_echo_int16(*data):
    data = defirmatify(data)
    print(decode_16(data, 0))

def on_echo_raw(*data):
    print(data)

def on_set_speed(*data):
    data = defirmatify(data)
    print(f'(Requested) Motor: {decode_8(data, 0, False)}, speed: {decode_16(data, 1)}')

def on_get_speed(*data):
    data = defirmatify(data)
    print(f'(Queried)   Motor: {decode_8(data, 0, False)}, speed: {decode_16(data, 1)}')

def on_send_step(*data):
    data = defirmatify(data)
    print(f'(Requested) Motor: {decode_8(data, 0, False)}, steps: {decode_16(data, 1, False)}, speed: {decode_16(data, 3)}')

def on_seek_position(*data):
    data = defirmatify(data)
    print(f'(Requested) Motor: {decode_8(data, 0, False)}, position: {decode_32(data, 1)}, speed: {decode_16(data, 5)}')

def on_get_position(*data):
    data = defirmatify(data)
    print(f'(Queried)   Motor: {decode_8(data, 0, False)}, position: {decode_32(data, 1)}')

# Setup Firmata
b = Arduino(COM_PORT)
b.add_cmd_handler(0x71, on_echo_text)
it = util.Iterator(b)
it.start()

# Setup handlers
b.add_cmd_handler(SYSEX_COMMAND_SET_SPEED, on_set_speed)
b.add_cmd_handler(SYSEX_COMMAND_GET_SPEED, on_get_speed)
b.add_cmd_handler(SYSEX_COMMAND_SEND_STEP, on_send_step)
b.add_cmd_handler(SYSEX_COMMAND_SEEK_POS, on_seek_position)
b.add_cmd_handler(SYSEX_COMMAND_GET_POS, on_get_position)

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
    b.send_sysex(SYSEX_COMMAND_GET_POS, firmatify(bytearray([motor])))
    b.send_sysex(SYSEX_COMMAND_SET_SPEED, firmatify(bytearray([motor]) + pack_16(20)))
    b.send_sysex(SYSEX_COMMAND_GET_SPEED, firmatify(bytearray([motor])))
    time.sleep(5)
    b.send_sysex(SYSEX_COMMAND_SET_SPEED,  firmatify(bytearray([motor]) + pack_16(-10)))
    b.send_sysex(SYSEX_COMMAND_GET_SPEED, firmatify(bytearray([motor])))
    time.sleep(5)
    b.send_sysex(SYSEX_COMMAND_SET_SPEED, firmatify(bytearray([motor]) + pack_16(0)))
    b.send_sysex(SYSEX_COMMAND_GET_SPEED, firmatify(bytearray([motor])))
    
    # Step forward 20 steps at 10 RPM, then back 10 steps at 5 RPM
    b.send_sysex(SYSEX_COMMAND_GET_POS, firmatify(bytearray([motor])))
    b.send_sysex(SYSEX_COMMAND_SEND_STEP, firmatify(bytearray([motor]) + pack_16(20) + pack_16(100)))
    time.sleep(1)
    b.send_sysex(SYSEX_COMMAND_GET_POS, firmatify(bytearray([motor])))
    b.send_sysex(SYSEX_COMMAND_SEND_STEP, firmatify(bytearray([motor]) + pack_16(10) + pack_16(-50)))
    time.sleep(1)
    b.send_sysex(SYSEX_COMMAND_GET_POS, firmatify(bytearray([motor])))
    
    time.sleep(1)
    
    # Seek back to position 0 from wherever we ended up at 10 RPM
    b.send_sysex(SYSEX_COMMAND_SEEK_POS, firmatify(bytearray([motor]) + pack_32(0) + pack_16(100)))
    time.sleep(1)
    b.send_sysex(SYSEX_COMMAND_GET_POS, firmatify(bytearray([motor])))
    
    time.sleep(1)

