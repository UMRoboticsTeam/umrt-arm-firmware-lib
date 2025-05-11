"""@file
This file lists the commands of the MKS SERVO57D/42D/35D/28D stepper motor driver modules.
"""

COM_PORT = '/dev/ttyACM3'
DEVICE_ADDR = 0x01

from enum import IntEnum
import can
import sys
import time
import regex


## Constants associated with MksTest.py.
# Organised in a class to get Doxygen to generate links properly.
class Constants:
    ## The maximum speed the driver can spin a motor, depending on the work mode.
    # The values are nominally:
    # | Work Mode | Max Speed [RPM] |          Work Mode Description           |
    # | :-------- | :-------------: | :--------------------------------------- |
    # |  CR_OPEN  |       400       | Open loop, pulse interface               |
    # |  SR_OPEN  |       400       | Open loop, serial interface              |
    # |  CR_CLOSE |      1500       | Closed loop, pulse interface             |
    # |  SR_CLOSE |      1500       | Closed loop, serial interface            |
    # |  CR_vFOC  |      3000       | Field oriented control, pulse interface  |
    # |  SR_vFOC  |      3000       | Field oriented control, serial interface |
    #
    # However, these values actually indicate the maximum step rate, computed under [nominal](@ref noc) conditions.
    # If another microstepping mode is used, the maximum speed will be adjusted.
    # Despite this, the motor speed cannot exceed 3000 RPM regardless of microstepping mode. TODO: Why?
    #
    # For example, if using CR_OPEN at quarter-stepping, the maximum speed is 1600 RPM instead of 400 RPM.
    #
    # Note that if a speed greater than one of these is used, the motor will instead spin at the applicable maximum.
    #
    # TODO: It is not clear from the manual whether *_CLOSE and *_vFOC were computed using 16-microstepping, or perhaps 32-
    #       and 64-microstepping respectively.
    MAX_SPEED = {
        0x00: 400,  # Open loop, pulse interface (CR_OPEN)
        0x03: 400,  # Open loop, serial interface (SR_OPEN)
        0x01: 1500,  # Closed loop, pulse interface (CR_CLOSE)
        0x04: 1500,  # Closed loop, serial interface (SR_CLOSE)
        0x02: 3000,  # Field oriented control, pulse interface (CR_vFOC)
        0x05: 3000,  # Field oriented control, serial interface (SR_vFOC)
    }
    
    ## The maximum acceleration ramp a motor can use.
    # When acceleration is set to 0, the motor instantaneously accelerates to the target speed.
    #
    # When not set to 0, the current speed of the motor increases by 1 RPM per tick, and the acceleration value controls the
    #   time between ticks.
    # Therefore, the speed of the motor is governed by:
    # \f[
    #   t_{next} - t_{now} = (256 - \alpha) * 50 [\mu s] \\
    #   v_{next} = \begin{cases}
    #               v_{now} + 1,   &   v_{now} \le v_{target} \\
    #               v_{now},       &   v_{now} = v_{target} \\
    #               v_{now} - 1,   &   v_{now} \gt v_{target}
    #              \end{cases}
    # \f]
    #
    # For example, to reach \f$v_{target}=5\f$ at \f$\alpha=246\f$:
    # | Time [ms] | Speed [RPM] |
    # | :-------: | :---------: |
    # |     0     |      0      |
    # |    0.5    |      1      |
    # |     1     |      2      |
    # |    1.5    |      3      |
    # |     2     |      4      |
    # |    2.5    |      5      |
    # |     3     |      5      |
    MAX_ACCEL = 255


class Commands(IntEnum):
    @staticmethod
    def checksum(device_id, payload):
        """MKS SERVO57D/42D/35D/28D drivers use an 8-bit sum-of-bytes checksum."""
        cs = device_id
        for n in payload:
            cs = (cs + n) & 0xFF
        return cs


def extract_mks_commands(enum_header):
    # Match a line beginning with whitespace, followed by a word and then " = 0x" and two hexadecimal digits
    # The word would be the command name, which is saved into a capture group
    # The two hex digits are the command, and are saved into another capture group
    REGEX_PATTERN = r"^\s*(\w+) = 0x([0-9A-Fa-f]{2})"
    
    with open(enum_header, 'r') as f:
        for line in f:
            match = regex.match(REGEX_PATTERN, line)
            if match:
                exec(f"{match[1]}=0x{match[2]}", globals())
                
    pass
        

def format_canmsg(msg: can.Message):
    """
    Formats a CAN message into the format used in the manual.
    The output is a list with the first entry the device address, followed by the message payload.
    :param msg: CAN message to format
    :return: msg formatted as a list
    """
    out = [msg.arbitration_id]
    out.extend(msg.data)
    return out


def squeeze_msg(msg: can.Message):
    """
    Formats a CAN message as @ref format_canmsg, but squeezes the list into a single integer.
    Useful for comparing to expected output without having to write out lists.
    E.g. using this a message can be compared to 0x01_FD_01_40_02_00_FA_00_3B instead of
        [0x01, 0xFD, 0x01, 0x40, 0x02, 0x00, 0xFA, 0x00, 0x3B].
    :param msg: CAN message to format
    :return: msg formatted as an int
    """
    preformatting = format_canmsg(msg)
    out = 0
    l = len(preformatting) - 1
    for i in range(len(preformatting)):
        out |= preformatting[i] << (8 * (l - i))  # We want big endian
    return out


def get_motor_speed(driver_can_id: int, bus: can.Bus = None):
    payload = [MOTOR_SPEED]
    
    # Calculate checksum
    payload.append(Commands.checksum(driver_can_id, payload))
    
    msg = can.Message(arbitration_id=driver_can_id,
                      data=payload,
                      is_extended_id=False)
    
    if bus is not None:
        try:
            bus.send(msg)
        except can.CanError:
            print("Error sending CAN message")
    return msg, None


def get_current_pos(driver_can_id: int, bus: can.Bus = None):
    payload = [CURRENT_POS]
    
    # Calculate checksum
    payload.append(Commands.checksum(driver_can_id, payload))
    
    msg = can.Message(arbitration_id=driver_can_id,
                      data=payload,
                      is_extended_id=False)
    
    if bus is not None:
        try:
            bus.send(msg)
        except can.CanError:
            print("Error sending CAN message")
    return msg, None


def set_speed(driver_can_id: int, dir: bool, speed: int, accel: int, bus: can.Bus = None, norm_factor: int = 16):
    payload = [SET_SPEED]

    # Speed is normalised when norm_factor is 16
    # That is, at 16 normalised_speed = speed
    # At 1, normalised_speed = speed / 16
    # At 32, normalised_speed = speed * 2
    normalised_speed = int(speed * norm_factor / 16)
    
    # Encode speed properties
    speed_properties_low = (normalised_speed & 0xF00) >> 8 | (dir & 0x1) << 7
    speed_properties_high = normalised_speed & 0xFF
    #speed_properties_low = (normalised_speed & 0x0F) | (dir & 0x1) << 7
    #speed_properties_high = normalised_speed & 0xFF0 >> 4
    payload.append(speed_properties_low)
    payload.append(speed_properties_high)
    
    # Encode acceleration
    payload.extend(accel.to_bytes(1, 'big'))
    
    # Calculate checksum
    payload.append(Commands.checksum(driver_can_id, payload))
    
    msg = can.Message(arbitration_id=driver_can_id,
                      data=payload,
                      is_extended_id=False)
    
    if bus is not None:
        try:
            bus.send(msg)
        except can.CanError:
            print("Error sending CAN message")
    return msg


def send_step(driver_can_id: int, dir: bool, speed: int, accel: int, steps: int, bus: can.Bus = None, norm_factor: int = 16):
    payload = [SEND_STEP]

    normalised_speed = int(speed * norm_factor / 16)
    normalised_steps = steps * norm_factor
    
    # Encode speed properties
    speed_properties_low = (normalised_speed & 0xF00) >> 8 | (dir & 0x1) << 7
    speed_properties_high = normalised_speed & 0xFF
    payload.append(speed_properties_low)
    payload.append(speed_properties_high)
    
    # Encode acceleration
    payload.extend(accel.to_bytes(1, 'big'))
    
    # Encode steps
    payload.extend(normalised_steps.to_bytes(3, 'big'))
    
    # Calculate checksum
    payload.append(Commands.checksum(driver_can_id, payload))
    
    msg = can.Message(arbitration_id=driver_can_id,
                      data=payload,
                      is_extended_id=False)
    
    if bus is not None:
        try:
            bus.send(msg)
        except can.CanError:
            print("Error sending CAN message")
    return msg


def seek_pos_by_steps(driver_can_id: int, speed: int, accel: int, pos: int, bus: can.Bus = None, norm_factor: int = 16):
    payload = [SEEK_POS_BY_STEPS]

    normalised_speed = int(speed * norm_factor / 16)
    normalised_position = pos * norm_factor
    
    # Encode properties
    payload.extend(normalised_speed.to_bytes(2, 'big'))
    payload.extend(accel.to_bytes(1, 'big'))
    payload.extend(normalised_position.to_bytes(3, 'big', signed=True))
    
    # Calculate checksum
    payload.append(Commands.checksum(driver_can_id, payload))
    
    msg = can.Message(arbitration_id=driver_can_id,
                      data=payload,
                      is_extended_id=False)
    
    if bus is not None:
        try:
            bus.send(msg)
        except can.CanError:
            print("Error sending CAN message")
    return msg


def on_motor_speed(msg):
    if msg is not None and len(msg.data) == 2 + 2:  # Length is code + crc + payload
        if msg.data[0] == MOTOR_SPEED:
            if Commands.checksum(driver_can_id, msg.data[:-1]) != msg.data[-1]:
                print(f"Checksum error in message: {msg}")
            
            speed = int.from_bytes(msg.data[1:3], 'little', signed=True)
            print(f"Speed: {speed}")


def on_current_pos(msg):
    if msg is not None and len(msg.data) == 2 + 4:  # Length is code + crc + payload
        if msg.data[0] == CURRENT_POS:
            if Commands.checksum(driver_can_id, msg.data[:-1]) != msg.data[-1]:
                print(f"Checksum error in message: {msg}")
            
            pos = int.from_bytes(msg.data[1:5], 'big', signed=True)
            print(f"Position: {pos}")


def test(driver_can_id, can_device, bitrate):
    # Compare to manual examples without sending actual commands
    # SET_SPEED
    print(0x01_F6_01_40_02_3A == squeeze_msg(set_speed(1, False, 320, 2)))
    print(0x01_F6_81_40_02_BA == squeeze_msg(set_speed(1, True, 320, 2)))
    # SEND_STEP
    print(0x01_FD_01_40_02_00_FA_00_3B == squeeze_msg(send_step(1, False, 320, 2, 20 * 200 * 16)))
    print(0x01_FD_81_40_02_00_FA_00_BB == squeeze_msg(send_step(1, True, 320, 2, 20 * 200 * 16)))
    # SEEK_POS_BY_STEP
    print(0x01_FE_02_58_02_00_40_00_9B == squeeze_msg(seek_pos_by_steps(1, 600, 2, 0x4000)))
    print(0x01_FE_02_58_02_FF_C0_00_1A == squeeze_msg(seek_pos_by_steps(1, 600, 2, -0x4000)))
    
    # Motor testing sequence
    with can.Bus(interface='socketcan', channel=can_device, bitrate=bitrate) as bus:
    #if True:
        # Register message handlers
        notifier = can.Notifier(bus, [])
        notifier.add_listener(on_motor_speed)
        notifier.add_listener(on_current_pos)

        set_speed(driver_can_id, False, 6, 0, bus, 16)
        #set_speed(1, False, 1, 2, bus)
        get_motor_speed(driver_can_id, bus)
        #get_current_pos(driver_can_id, bus)
        time.sleep(5)
        get_motor_speed(driver_can_id, bus)
        #get_current_pos(driver_can_id, bus)
        time.sleep(4)
        get_motor_speed(driver_can_id, bus)
        #get_current_pos(driver_can_id, bus)
        time.sleep(1)
        set_speed(driver_can_id, False, 0, 0, bus)
        time.sleep(5)
        #
        # set_speed(driver_can_id, True, 2, 0, bus)
        # get_motor_speed(driver_can_id, bus)
        # get_current_pos(driver_can_id, bus)
        # time.sleep(2)
        # get_motor_speed(driver_can_id, bus)
        # get_current_pos(driver_can_id, bus)
        # time.sleep(2)
        # get_motor_speed(driver_can_id, bus)
        # get_current_pos(driver_can_id, bus)
        # time.sleep(1)
        # set_speed(driver_can_id, False, 0, 0, bus)
        # time.sleep(5)

        # set_speed(1, False, 32, 2, bus)
        # get_motor_speed(driver_can_id, bus)
        # time.sleep(5)
        # set_speed(1, False, 0, 2, bus)

        # seek_pos_by_steps(driver_can_id, 1, 0, 200*8, bus)
        # get_motor_speed(driver_can_id, bus)
        # get_current_pos(driver_can_id, bus)
        # time.sleep(2)
        # get_motor_speed(driver_can_id, bus)
        # get_current_pos(driver_can_id, bus)
        # time.sleep(2)
        # get_motor_speed(driver_can_id, bus)
        # get_current_pos(driver_can_id, bus)
        # time.sleep(1)

        # Send speed of 2 RPM for 5 seconds, then 1 RPM in other direction for 5 seconds, then stop
        # get_current_pos(driver_can_id, bus)
        # set_speed(driver_can_id, False, 2, 0, bus)
        # get_motor_speed(driver_can_id, bus)
        # time.sleep(5)
        # set_speed(driver_can_id, True, 1, 0, bus)
        # get_motor_speed(driver_can_id, bus)
        # time.sleep(5)
        # set_speed(driver_can_id, False, 0, 0, bus)
        # get_motor_speed(driver_can_id, bus)
        #
        # # Step forward 20 steps at 10 RPM, then back 10 steps at 5 RPM
        # get_current_pos(driver_can_id, bus)
        # send_step(driver_can_id, False, 10, 0, 20, bus)
        # time.sleep(1)
        # get_current_pos(driver_can_id, bus)
        # send_step(driver_can_id, True, 5, 0, 20, bus)
        # time.sleep(1)
        # get_current_pos(driver_can_id, bus)
        #
        # time.sleep(1)
        #

        # Step one full rotation in 5 seconds
        send_step(driver_can_id, False, 12, 0, 200, bus, 128)
        time.sleep(6)


        # Seek back to position 0 from wherever we ended up at 10 RPM
        #seek_pos_by_steps(driver_can_id, 10, 0, 0, bus)
        seek_pos_by_steps(driver_can_id, 10, 0, 0, bus)
        time.sleep(1)
        get_current_pos(driver_can_id, bus)
        
        time.sleep(1)
        notifier.stop()


if __name__ == "__main__":
    can_device = sys.argv[1]
    driver_can_id = sys.argv[2] if len(sys.argv) >= 3 else 0x01
    bitrate = sys.argv[3] if len(sys.argv) >= 4 else 500_000
    extract_mks_commands("../include/umrt-arm-firmware-lib/MKS_COMMANDS.hpp")
    test(driver_can_id, can_device, bitrate)
