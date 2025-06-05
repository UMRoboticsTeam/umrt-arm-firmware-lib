//
// Created by Noah on 2025-04-23.
//
/**
* @file
* Describes the commands which are available to send MKS SERVO57D/42D/35D/28D stepper motor driver modules.
*/

#ifndef UMRT_ARM_FIRMWARE_LIB_MKS_COMMANDS_HPP
#define UMRT_ARM_FIRMWARE_LIB_MKS_COMMANDS_HPP

#include <cstdint>

/** CAN commands for the MKS SERVO57D/42D/35D/28D stepper motor driver modules.
 * Each message must have its checksum appended.
 * Uses big-endian encoding.
 *
 * The checksum function is a simple 8-bit sum-of-bytes. A sample Python implementation is:
 * @code{.py}
 * cs = 0
 * for x in message:
 *     cs = (cs + x) & 0xFF
 * return cs
 * @endcode
 *
 *
 * # Nominal Operating Conditions {#noc}
 * When used, "nominal conditions" refers to a stepper driver configured to be in 16-microstepping mode along
 * with a 200 step/rev motor.
 * These are the conditions the manual authors assumed, and thus many hard-coded computations are based on it.
 * Particularly, the manual documents the motor speed units to be "RPM".
 * This is only true for these conditions; a speed of "1 RPM" actually corresponds to 3200 steps per minute.
 * If a 200 step/rev motor is being used in full-step mode, a speed command of "1 RPM" will instead move the
 * motor at \f$v = 1 * \left(\frac{3200\ steps}{min}\right) \left(\frac{1\ full-step}{step}\right) \left(\frac{1\ rev}{200\ full-steps}\right) = 16\ RPM\f$
 */
enum MksCommands : uint8_t {
    /** Encoder value, split into number of turns and angle.
     * The angle is a 14-bit unsigned integer (0-0x3FFF), and follows the formula:<br>
     * &emsp;    angle = 360 * angle_value / 2^14
     *
     * For example, a quarter turn CW would be (0, 0x1000), and a quarter turn CCW would be (-1, 0x3000).
     *
     * Note that the manual refers to number of turns as the "carry".
     *
     * @return num_turns     [int32] the number of turns from the zero point
     * @return angle_value   [uint16] the angle, in 1/2^14 of a rotation
     *
     */
    ENCODER_SPLIT = 0x30,

    /** Encoder value, with the number of turns and angle combined into a single integer.
     * The angle is a 48-bit signed integer (0-0x3FFF for one CW rotation), and follows the formula:<br>
     * &emsp;   angle = 360 * combined_angle_value / 2^14
     *
     * For example, a quarter turn CW would be 0x1000, and a quarter turn CCW would be -(0x1000)=0xFFFF_FFFF_F000
     *
     * @return combined_angle_value [int48] the angular offset from the sero point, in 1/2^14 of a rotation
     */
    ENCODER_ADDITIVE = 0x31,

    /** Motor speed, as determined by the encoder.
     * Represented as a signed integer, where positive denotes CCW and negative denotes CW.
     *
     * @return speed [int16] the current speed in RPM
     * TODO: Units probably wrong
     * TODO: I am assuming it is determined by the encoder because I am seeing odd responses, and I don't have an encoder installed
     */
    MOTOR_SPEED = 0x32,

    /** The current position of the motor, in steps
     * As an alternative to the encoder value, this command allows position tracking by step counting.
     *
     * @return steps [int32] the angular offset from the sero point, in steps
     */
    CURRENT_POS = 0x33,

    /** Status of the IO ports.
     * Returns a byte, with each bit representing:
     * | Bit     |   7   |   6  |   5   |   4   |      3       |      2       |      1      |      0      |
     * | :------ | :---: | :---:| :---: | :---: | :----------: | :----------: | :---------: | :---------: |
     * | Meaning |   Manufacturer Reserved   |||| OUT_2 status | OUT_1 status | IN_1 status | IN_2 status |
     *
     * Note that if LIMIT_PORT_REMAP is enabled, IN_1 maps to En, and IN_2 maps to Dir
     *
     * @return flags [uint8] the port statuses, formatted as above
     */
    IO_STATUS = 0x34,

    /** Encoder value, in the "raw" (?) format.
     * The manual entry for this is identical to ENCODER_ADDITIVE.
     *
     * @return combined_angle_value  [int48]
     */
    ENCODER_RAW = 0x35,

    /** Difference between the target angle (mod 1 turn) and the current angle.
     * The error is a signed integer, where the range 0-0xC800 maps to 0-360°.
     *
     * @return error_value   [int32] the error, calculated as above
     * TODO: Confirm it is mod 1
     */
    TARGET_ANGLE_ERROR = 0x39,

    /** Status of the En pin.
     *
     * @return enabled   [uint8]
     */
    ENABLE_STATUS = 0x3A,

    /** Whether the motor successfully went back to the zero point once powered on.
     * There are three legal return values:
     * <ul style="list-style-type:none;">
     *   <li>0x00: The motor is currently moving towards the zero point</li>
     *   <li>0x01: The motor successfully reached the zero point when started</li>
     *   <li>0x02: The motor failed to reach the zero point when started</li>
     * </ul>
     *
     * @return status    [uint8] the status code
     */
    GO_HOME_STATUS = 0x3B,

    /** Commands the driver to release a motor from the locked-rotor protection state.
     *
     * @return succeeded [uint8] 1 if successfully set, 0 otherwise
     */
    RELEASE_SHAFT_LOCK = 0x3D,

    /** Whether the motor is currently in the locked-rotor protection state.
     * Note that the manual says "protected" vs. "no protected", should be confirmed before use.
     *
     * @return locked    [uint8] 1 if locked, 0 otherwise
     */
    SHAFT_LOCK_STATUS = 0x3E,

    /** Calibration status.
     * The motor must be unloaded for calibration.
     * There are three legal return values:
     * <ul style="list-style-type:none;">
     *     <li>0x00: The motor is currently calibrating</li>
     *     <li>0x01: Motor calibration succeeded</li>
     *     <li>0x02: Motor calibration failed</li>
     * </ul>
     * Accessible through the driver screen as "Cal".
     *
     * @param begin      [uint8] set to 0x00 to begin calibration
     * @return status    [uint8] the status code
     */
    CALIBRATION = 0x80,

    /** Sets the work mode of the motor driver.
     * There are 6 possible options: (Default: CR_vFOC)
     * <ul style="list-style-type:none;">
     *     <li>0x00: Open loop, pulse interface (CR_OPEN)</li>
     *     <li>0x01: Closed loop, pulse interface (CR_CLOSE)</li>
     *     <li>0x02: Field oriented control, pulse interface (CR_vFOC)</li>
     *     <li>0x03: Open loop, serial interface (SR_OPEN)</li>
     *     <li>0x04: Closed loop, serial interface (SR_CLOSE)</li>
     *     <li>0x05: Field oriented control, serial interface (SR_vFOC)</li>
     * </ul>
     * Accessible through the driver screen as "Mode"
     *
     * @param mode       [uint8] the requested work mode
     * @return succeeded [uint8] 1 if successfully set, 0 otherwise
     */
    SET_WORK_MODE = 0x82,

    /** Sets the maximum current the driver is allowed to push through the motor (working current).
     * Should not exceed 3000mA for SERVO42D/35D/28D models, and should not exceed 5200mA for the SERVO57D model.
     *
     * Accessible through the driver screen as "Ma".
     *
     * @param current    [uint16] working current, in milliamps
     * @return succeeded [uint8] 1 if successfully set, 0 otherwise
     */
    SET_WORKING_CURRENT = 0x83,

    /** Sets the percentage of the working current to use for as the maximum holding current.
     * I am not entirely clear on what this means for a stepper motor, but that is what the manual says.
     * This is set as a value from 0-8:
     * <ul style="list-style-type:none;">
     *     <li>0x00: 10%</li>
     *     <li>0x01: 20%</li>
     *     <li>0x02: 30%</li>
     *     <li>...</li>
     *     <li>0x08: 90%</li>
     * </ul>
     * Note that this is not applicable to vFOC modes.
     *
     * Accessible through the driver screen as "HoldMa".
     *
     * @param percentage_mode    [uint8] holding current percentage, according to the scale above
     * @return succeeded         [uint8] 1 if successfully set, 0 otherwise
     */
    SET_HOLDING_CURRENT = 0x83,

    /** Sets the microstepping division.
     * Can be set to any division, not just powers of 2.
     *
     * Accessible through the driver screen as "MStep", though only 1, 2, 4, 8, 16, 32, 64, 128, and 256 are available.
     *
     * @param mode       [uint8] microsteps per fullstep, with 0x00 meaning 256-microstepping
     * @return succeeded [uint8] 1 if successfully set, 0 otherwise
     */
    SET_MICROSTEP = 0x84,

    /** Sets the En pin mode.
     * There are three possible modes:
     * <ul style="list-style-type:none;">
     *     <li>0x00: active low (L on driver screen)</li>
     *     <li>0x01: active high (H on driver screen)</li>
     *     <li>0x02: active always (Hold on driver screen)</li>
     * </ul>
     * Accessible through the driver screen as "En".
     *
     * @param mode       [uint8] pin mode
     * @return succeeded [uint8] 1 if successfully set, 0 otherwise
     */
    SET_EN_MODE = 0x84,

    /** Sets the Dir pin mode.
     * There are three possible modes: (These should be confirmed before being used, manual is somewhat ambiguous)
     * <ul style="list-style-type:none;">
     *     <li>0x00: CW when low</li>
     *     <li>0x01: CCW when low</li>
     * </ul>
     * Only applicable when using the pulse interface.
     *
     * Accessible through the driver screen as "Dir".
     *
     * @param mode       [uint8] pin mode
     * @return succeeded [uint8] 1 if successfully set, 0 otherwise
     */
    SET_DIR_MODE = 0x86,

    /** Enables automatic shutoff of the driver screen after ~15 seconds.
     * The screen can be woken up by pressing any button.
     *
     * Accessible through the driver screen as "AutoSDD".
     *
     * @param enable     [uint8] 1 to enable, 0 to disable
     * @return succeeded [uint8] 1 if successfully set, 0 otherwise
     */
    ENABLE_DISPLAY_SLEEP = 0x86,

    /** Lock/unlock the rotor.
     * Note that the motor can be unlocked by sending @ref RELEASE_SHAFT_LOCK, pressing "Enter" on the driver screen, or
     *     pulsing the En pin if in pulse interface mode.
     *
     * Accessible through the driver screen as "Protect".
     *
     * @param lock       [uint8] 1 to enable, 0 to disable
     * @return succeeded [uint8] 1 if successfully set, 0 otherwise
     * TODO: Is this locking, or setting the driver to somehow automatically lock?
     */
    ENABLE_ROTOR_LOCK = 0x88,

    /** Enable microstep "interpolation", which internally uses 256-microstepping for smoother movement.
     * Works by internally setting the motor microstep subdivisions to 256 instead of the division x set by @ref SET_MICROSTEP,
     *     and converting each x-microstep into the equivalent number of 256-microsteps each time the motor is stepped.
     *
     * For example, if this is used with quarter-stepping, the motor will actually move 64 256-microsteps each time.
     *
     * This allegedly reduces the vibration and noise when the motor moves at low speed.
     *
     * Accessible through the driver screen as "Mplyer".
     *
     * @param enable     [uint8] 1 to enable, 0 to disable
     * @return succeeded [uint8] 1 if successfully set, 0 otherwise
     */
    ENABLE_MICROSTEP_INTERPOLATION = 0x89,

    /** Sets the CAN bus baud rate.
     * There are four possible options:
     * <ul style="list-style-type:none;">
     *     <li>0x00: 125 K</li>
     *     <li>0x01: 250 K</li>
     *     <li>0x02: 500 K</li>
     *     <li>0x03: 1000 K</li>
     * </ul>
     * Accessible through the driver screen as "CanRate".
     *
     * @param baud_rate  [uint8] the selected CAN baud rate
     * @return succeeded [uint8] 1 if successfully set, 0 otherwise
     */
    CAN_BAUD_RATE = 0x8A,

    /** Sets the CAN bus ID for this driver.
     * Eligible IDs are 0x001-0x7FF.
     * Must not be set to 0 as that is the broadcast address.
     *
     * Accessible through the driver screen as "CanID".
     *
     * @param new_id     [uint16] the requested CAN bus ID
     * @return succeeded [uint8] 1 if successfully set, 0 otherwise
     */
    CAN_ID = 0x8B,

    /** Change response mode.
     *
     * There are three possible response modes:
     * | Mode     | Description                                                                                   |
     * | :------- | :-------------------------------------------------------------------------------------------- |
     * | Disabled | The driver will not send message responses                                                    |
     * |  ^       | TODO: Does this apply to read requests too?                                                   |
     * | Passive  | The driver will immediately respond to commands, and not send any follow-ups                  |
     * | Active   | The driver will immediately respond to commands, and send follow-up information as applicable |
     *
     * Some commands are capable of sending multiple responses.
     * For example, @ref SEND_STEP could send one response with 0x01 indicating the motor has begun moving, followed by
     *   another response with 0x02 indicating the motor has reached the target.
     * Alternatively, it may only respond with 0x00 indicating that the command has been rejected.
     * In this case, in active mode all of these responses could be sent, whereas in passive mode only the first response
     *   would be.
     *
     * @param enable_responses   [uint8] set to 0 to disable responses, or 1 to enable
     * @param allow_active       [uint8] provided enable_responses is 1, set to 0 to enter passive mode or 1 for active
     */
    RESPONSE_MODE = 0x8C,

    /** Disables the buttons on the driver.
     *
     * @param disable    [uint8] 1 to disable keys, 0 to enable
     * @return succeeded [uint8] 1 if successfully set, 0 otherwise
     */
    DISABLE_BUTTONS = 0x8F,

    /** Sets the group ID of this driver.
     * Commands sent to the group ID will apply to every driver with that group ID.
     * The group ID acts as an alternative CAN bus ID.
     * However, no drivers will send responses to messages sent to their group ID.
     *
     * For example, could be used to link the left-side front and rear motors of differential drive car.
     * Must not be set to 0 as that is the broadcast address.
     *
     * Eligible IDs are 0x001-0x7FF.
     *
     * @param group_id   [uint16] the requested group ID
     * @return succeeded [uint8] 1 if successfully set, 0 otherwise
     * TODO: Make sure I understand the manual's example
     */
    SET_GROUP_ID = 0x8D,

    /** Write values to the IO ports.
     * Input is a byte, with each bit representing:
     * | Bit     |   7   |   6   |   5   |   4   |      3      |      2      |  1  |  0  |
     * | :------ | :---: | :---: | :---: | :---: | :---------: | :---------: | :-: | :-: |
     * | Meaning |  OUT_2 Mode  ||  OUT_1 mode  || OUT_2 value | OUT_1 value |  0  |  0  |
     *
     * There are 3 possible modes for OUT_2 and OUT_1:
     * <ul style="list-style-type:none;">
     *     <li>0b00: Do not write to the port</li>
     *     <li>0b01: Write passed OUT_1/OUT_2 value to the port</li>
     *     <li>0b02: Do not change the port's value</li>
     * </ul>
     *
     * @param settings   [uint8] flags byte following the above format
     * @return succeeded [uint8] 1 if successfully set, 0 otherwise
     * TODO: What is the difference between 0b00 and 0b02?
     */
    WRITE_IO = 0x36,

    /** Sets parameters for the return-to-home sequence.
     * Allows the following parameters to be changed:
     * - Home trigger: The logic level of the end stop input, which is used to determine when the motor has reached home
     * - Home direction: Whether home is CW or CCW from the motor
     * - Home speed: How fast the motor should move during the homing sequence
     * - End stop lock: Whether to automatically override the rotor shaft lock generated when the motor reaches the end stop
     * - Mode: Whether to use the end stop input to determine when home is reached, or the technique described by
     *         @ref SET_BLIND_LIMIT
     *
     * The manual states that it is necessary to issue a homing command after the settings have been changed.
     *
     * @param home_trigger [uint8] 1 for active high or 0 for active low
     * @param home_dir     [uint8] the direction of home, 1 for CCW or 0 for CW
     * @param home_speed   [uint16] speed to move towards home at, in units of 160/3 steps/s, see @ref Constants.MAX_SPEED for restrictions
     * @param lock_at_end  [uint8] 1 to automatically clear the rotor shaft lock after homing, 0 to not
     * @param mode         [uint8] 1 to use the blind-limit-sensing or 0 to use the end stop input
     * @return succeeded   [uint8] 1 if successfully set, 0 otherwise
     * TODO: Confirm the logic levels are not backwards
     */
    HOME_SETTINGS = 0x90,

    /** Start the homing sequence.
     * Note that if the limit switch is already triggered, the motor will move in the opposite direction of home until
     *   the limit switch releases, at which point to will restart the homing sequence.
     *
     * There are three legal return values:
     * <ul style="list-style-type:none;">
     *     <li>0x00: The motor failed to return home</li>
     *     <li>0x01: The motor is starting the homing sequence</li>
     *     <li>0x02: The motor has successfully homed</li>
     * </ul>
     * @return status    [uint8] the status code
     */
    GO_HOME = 0x91,

    /** Set the zero position.
     * Sets the motor's current position to 0.
     *
     * @return succeeded [uint8] 1 if successfully set, 0 otherwise
     */
    SET_ZERO = 0x92,

    /** Blindly finds the axis limit by turning until the motor hits an obstacle.
     * This is an alternative to a physical limit switch for the homing procedure.
     *
     * The maximum distance to rotate is specified in increments of 1/2^14 of a rotation, the same as described in
     *   @ref ENCODER_SPLIT.
     * Therefore, to move at most 360°, a distance of 0x4000 should be used
     *
     * An alternate maximum current is set here to use when homing so the motor stalls before inflicting damage.
     * It would be a good idea to set the current limit at the minimum current required to move the motor, since it is
     *   going to hit something with whatever torque that current corresponds to.
     *
     * @param max_distance   [uint32] the maximum distance to rotate towards home, in 1/2^14 of a rotation
     * @param max_current    [uint16] the maximum current to use when homing, in milliamps
     * @return succeeded     [uint8] 1 if successfully set, 0 otherwise
     */
    SET_BLIND_LIMIT = 0x94,

    /** Remap ports for an additional limit or to facilitate wiring.
     * When used with the SERVO42D/35D/28D models, this converts the DIR port becomes the IN_2 port which is used as
     *   another end stop.
     * Without this, these models can only have one end stop.
     * Note that since there is no longer a DIR port, the pulse interface cannot be used.
     *
     * For the SERVO52D model, this switches the EN port with the IN_1 port, and the DIR port with the IN_2 port.
     * This may be helpful to facilitate wiring.
     *
     * @param enable_remap [uint8] 1 to enable, 0 to disable
     * @return succeeded   [uint8] 1 if successfully set, 0 otherwise
     */
    SET_LIMIT_PORT_REMAP = 0x9E,

    /** Set how the motor homes upon startup.
     * Allows the following parameters to be changed:
     * - Home mode: Whether to not home, move towards home in the specified direction, or move towards home in the
     *              direction which minimises the distance (the minimum angle)
     * - Enable: The manual lists options as clean zero, set zero, or do not modify zero; I do not understand what these mean
     * - Speed: A relative speed setting ranging from 0x00-0x04, where 0x04 is fastest and 0x00 is slowest
     * - Direction: Whether to move CW or CCW when the directional homing mode is used
     *
     * Home mode is accessible through the driver screen as "0_Mode", Enable as "Set 0", Speed as "0_Speed", and
     * Direction as "0_Dir".
     *
     * Note that the motor will not move beyond 359°.
     *
     * @param home_mode  [uint8] 0 to disable, 1 for directional, or 2 for minimum angle
     * @param enable     [uint8] 0 for clean zero, 1 for set zero, or 2 to do not modify zero
     * @param speed      [uint8] a value from 0x00-0x04, where 0x04 is fastest and 0x00 is slowest
     * @param dir        [uint8] direction to seek if applicable, 0 for CW or 1 for CCW
     * @return succeeded [uint8] 1 if successfully set, 0 otherwise
     */
    POWER_ON_HOMING_MODE = 0x9A,

    /** Restores the default settings.
     * After restoring factory settings, the driver will reboot and the motor will need to be calibrated.
     * The "Next" button on the driver screen may need to be pressed after sending this command. TODO: Confirm.
     *
     * @return succeeded [uint8] 1 if successfully restored, 0 otherwise
     */
    FACTORY_RESET = 0x3F,

    /** Reboots the motor driver.
     *
     * @return succeeded [uint8] 1 if successfully set, 0 otherwise
     */
    REBOOT_DRIVER = 0x41,

    /** Modifies motor protection settings and allows delayed homing when using the pulse interface.
     * This command allows two distinct functions to be enabled.
     *
     * The first affects the driver's zeroing behaviour.
     * @ref POWER_ON_HOMING_MODE can be used to automatically home the motor when power is restored, however in some
     *   applications this may not be safe.
     * When using the pulse interface, a @ref GO_HOME command cannot be sent to trigger the homing sequence.
     * Through the `en_trigger` flag, the driver can be programmed to instead begin homing when a 200ms pulse is received
     *   on the En pin.
     * Note that @ref POWER_ON_HOMING_MODE must be programmed for this to occur as the driver needs to know which
     *   direction, speed, etc. to move when the sequence is triggered.
     *
     * The second affects the driver's motor protection system.
     * If the `pos_error_protection` flag is enabled, the driver will enter the locked rotor shaft state if the position
     *   error is greater than a certain threshold within a time window.
     * I am not entirely clear on what this means, but the manual states that: "within x time, if the motor position
     * error is greater than y, the protection is started".
     * My interpretation is that "errors" refers to missed steps, and the goal is to enter the protection state when a
     *   certain number of steps are missed within a certain timeframe.
     * For example, an application may find it  beneficial to lock the motor if 5 steps are missed in 100ms.
     * The manual states that "when Errors = 28000, the motor is misaligned by 360°", but I do not understand what this
     *   means or if it contradicts the above.
     *
     * If the position protection system is triggered, the driver screen displays "Wrong2".
     * Note that "Wrong" is displayed on the screen when the stall protection system triggered.
     *
     * The flags are encoded into a byte according to:
     * | Bit     |  7  |  6  |  5  |  4  |  3  |  2  |      1       |            0            |
     * | :------ | :-: | :-: | :-: | :-: | :-: | :-: | :----------: | :---------------------: |
     * | Meaning |  Set to 0                    |||||| `en_trigger` | `poss_error_protection` |
     *
     * @param settings   [uint8] flags byte following the above format
     * @param time       [uint16] size of the position error protection time window, in ~15ms increments
     * @param errors     [uint16] error threshold for position error protection?
     *
     */
    SET_MISC_SETTINGS = 0x9D,

    /** Reads the current setting for a parameter.
     * Parameters are returned in the same format as they are set.
     * If a parameter does not support reading, the payload 0xFF_FF is returned.
     * From my interpretation of the manual, I suspect the only parameters which support reading are:
     * - @ref SET_WORK_MODE
     * - @ref SET_WORKING_CURRENT
     * - @ref SET_HOLDING_CURRENT
     * - @ref SET_MICROSTEP
     * - @ref SET_EN_MODE
     * - @ref SET_DIR_MODE
     * - @ref ENABLE_DISPLAY_SLEEP
     * - @ref ENABLE_ROTOR_LOCK
     * - @ref ENABLE_MICROSTEP_INTERPOLATION
     * - @ref CAN_BAUD_RATE
     * - @ref CAN_ID
     * - @ref RESPONSE_MODE
     * - @ref DISABLE_BUTTONS
     * - @ref SET_GROUP_ID
     *
     * For example, when used with SET_WORK_MODE the response would be a uint8 that could be compared to
     * the table described in SET_WORK_MODE to determine which mode the driver has been put in.
     *
     * @param parameter_code     [uint8] the code for the requested parameter
     * @return parameter_value   [unknown length] the requested parameter's value, or 0xFFFF if unavailable
     */
    READ_PARAM = 0x00,

    /** Get current motor status
     * Retrieves a status code describing the current state of the motor:
     * <ul style="list-style-type:none;">
     *   <li>0x00: Query failed</li>
     *   <li>0x01: Motor stopped</li>
     *   <li>0x02: Motor accelerating</li>
     *   <li>0x03: Motor slowing down</li>
     *   <li>0x04: Motor moving at target speed</li>
     *   <li>0x05: Motor homing</li>
     *   <li>0x06: Motor calibrating</li>
     * </ul>
     *
     * @return status   [uint8] status code
     */
    QUERY_STATUS = 0xF1,

    /** Enable the motor
     * Not entirely sure when this is needed.
     *
     * @param enable     [uint8] 1 to enable, 0 to disable
     * @return succeeded [uint8] 1 if successfully set, 0 otherwise
     */
    ENABLE_MOTOR = 0xF3,

    /** Immediately stop the motor.
     * Does not decelerate, attempts to immediately stops.
     * According to the manual, if a motor is moving at greater than 1000 RPM it is not advisable to immediately stop it.
     *
     * @return succeeded [uint8] 1 if stop successfully issued, 0 otherwise
     */
    EMERGENCY_STOP = 0xF7,

    /** Run the motor at a target speed.
     * Sets the motor to spin at a specific speed with a fixed acceleration.
     *
     * The speed properties are encoded into a 16-bit integer according to:
     * | Bit     |    15-8   |  7  |   6-4    |  3  |  2  |  1  |  0  |
     * | :------ | :-------: | :-: | :------: | :-: | :-: | :-: | :-: |
     * | Meaning | Speed_low | Dir | Reserved |     Speed_high     ||||
     *
     * The speed value is a 12-bit unsigned integer, split so that the low nibble of the word contains the high nibble of
     * the speed, and the high byte of the word contains the low byte of the speed. It can be reconstructed according to:<br>
     * &emsp;    Speed = Speed_low | (Speed_high << 8)
     *
     * For why the speed is encoded this way, note that this can be alternatively expressed as:
     * |  Byte   |         1          |||  2  |
     * | :------ | :----------------: ||| :-: |
     * |  Bits   |  7  |   6-4    | 3-0 | 7-0 |
     * | Meaning | Dir | Reserved |  Speed   ||
     *
     * For example, 0x81_40 indicates moving CW at v = 0x40 | (0x01 << 8) = 320 * (160/3 steps/s) = 51 200 steps/s [320 [nominal](@ref noc) RPM].<br>
     * For another, 0x0A_BC indicates moving CCW at v = 0xBC | (0x0A << 8) = 2748 * (160/3 steps/s) = 146 560 steps/s [2748 [nominal](@ref noc) RPM].
     *
     * Note that the motor can be stopped by sending a speed of 0.
     * If an acceleration of 0 is used, the motor will stop, or more generally reach the target speed, immediately.
     *
     * Dir: 1 indicates CW, and 0 CCW<br>
     * Speed: motor speed in units of 160/3 steps/s, see @ref Constants.MAX_SPEED for restrictions.
     *
     * @param packed_properties  [uint16] speed properties encoded as described above
     * @param acceleration       [uint8] motor acceleration, see @ref Constants.MAX_ACCEL for description
     * @return succeeded         [uint8] 1 if command successfully issued, 0 otherwise
     */
    SET_SPEED = 0xF6,

    /** Set the driver to engage at the current speed upon startup.
     * Saves the current speed and direction into memory.
     * When the driver starts up, it will begin spinning at this speed.
     *
     * @param command [uint8] 0xC8 to save the current speed, 0xCA to reset to 0
     * @return succeeded [uint8] 1 if successfully set, 0 otherwise
     */
    SET_POWER_ON_SPEED = 0xFF,

    /** Run the motor a specific number of steps.
     * The manual refers to this as "Position mode1: relative motion by pulses".
     * Commands the motor to move a certain number of steps at a target speed with a fixed acceleration.
     *
     * See @ref SEND_ANGLE for a variation of this command which moves by an angle instead of a number of steps.
     *
     * The speed properties are encoded into a 16-bit integer according to:
     * | Bit     |    15-8   |  7  |   6-4    |  3  |  2  |  1  |  0  |
     * | :------ | :-------: | :-: | :------: | :-: | :-: | :-: | :-: |
     * | Meaning | Speed_low | Dir | Reserved |     Speed_high     ||||
     *
     * The speed value is a 12-bit unsigned integer, split so that the low nibble of the word contains the high nibble of
     * the speed, and the high byte of the word contains the low byte of the speed. It can be reconstructed according to:<br>
     * &emsp;    Speed = Speed_low | (Speed_high << 8)
     *
     * Dir: 1 indicates CW, and 0 CCW<br>
     * Speed: target motor speed in units of 160/3 steps/s, see @ref Constants.MAX_SPEED for restrictions<br>
     * For additional information about encoding, see @ref SET_SPEED.
     *
     * The motor can be stopped by sending another command with zero packed_properties and steps.
     * Note that this is different then simply sending another SEND_STEP command, as usually the commands are queued so
     *   that the first command would finish and then the second would be executed.
     * The stop variation behaves differently, in that it is not queued and the motor immediately starts slowing.
     * If an acceleration of 0 is used, the motor will stop, or more generally reach the target speed, immediately.
     * Note as well that this may cause the motor to overshoot the original target if the acceleration is too slow.
     *
     * There are four legal return values:
     * <ul style="list-style-type:none;">
     *     <li>0x00: Movement failed</li>
     *     <li>0x01: The motor is moving</li>
     *     <li>0x02: The motor has reached the target position</li>
     *     <li>0x03: An end limit has been reached</li>
     * </ul>
     *
     * @param packed_properties  [uint16] speed properties encoded as described above
     * @param acceleration       [uint8] motor acceleration, see @ref Constants.MAX_ACCEL for description
     * @param steps              [uint24] number of steps to move
     * @return status            [uint8] the status code described above
     */
    SEND_STEP = 0xFD,

    /** Run the motor to a specific position in steps.
     * The manual refers to this as "Position mode2: absolute motion by pulses".
     * Commands the motor to a particular position at a target speed with a fixed acceleration.
     * As with @ref ENCODER_ADDITIVE, a positive position indicates CW of zero, and negative CCW of zero.
     *
     * See @ref SEEK_POS_BY_ANGLE for a variation of this command which takes the position as an angle instead of a
     *   number of steps.
     *
     * The motor can be stopped by sending another command with zero speed and steps.
     * Note that this is different from simply sending another SEEK_POS_BY_STEPS command, as usually the commands are
     *   queued so that the first command would finish and then the second would be executed.
     * The stop variation behaves differently, in that it is not queued and the motor immediately starts slowing.
     * If an acceleration of 0 is used, the motor will stop, or more generally reach the target speed, immediately.
     * Note as well that this may cause the motor to overshoot the original target if the acceleration is too slow.
     *
     * There are four legal return values:
     * <ul style="list-style-type:none;">
     *     <li>0x00: Movement failed</li>
     *     <li>0x01: The motor is moving</li>
     *     <li>0x02: The motor has reached the target position</li>
     *     <li>0x03: An end limit has been reached</li>
     * </ul>
     *
     * @param speed          [uint16] target motor speed in units of 160/3 steps/s, see @ref Constants.MAX_SPEED for restrictions
     * @param acceleration   [uint8] motor acceleration, see @ref Constants.MAX_ACCEL for description
     * @param position       [int24] position to move to, in steps from the zero point
     * @return status        [uint8] the status code described above
     */
    SEEK_POS_BY_STEPS = 0xFE,

    /** Run the motor CW or CCW by a specific angle.
     * Commands the motor to a particular position at a target speed with a fixed acceleration.
     * The manual refers to this as "Position mode3: relative motion by axis [angle]".
     * Angles are represented in the same fashion as @ref ENCODER_ADDITIVE, where a positive position indicates CW of
     *   zero, and negative CCW of zero.
     *
     * For example, if the motor is currently at 0x4000 (1 turn CW of zero), and 0xFFFF_FFFF_F000 (90° CCW) is sent, the
     *   motor will move to 0x3000 (270° CW of zero).
     *
     * See @ref SEND_STEP for a variation of this command which moves by a number of steps instead of an angle.
     *
     * The motor can be stopped by sending another command with zero speed and angle.
     * Note that this is different from simply sending another SEND_ANGLE command, as usually the commands are queued so
     *   that the first command would finish and then the second would be executed.
     * The stop variation behaves differently, in that it is not queued and the motor immediately starts slowing.
     * If an acceleration of 0 is used, the motor will stop, or more generally reach the target speed, immediately.
     * Note as well that this may cause the motor to overshoot the original target if the acceleration is too slow.
     *
     * There are four legal return values:
     * <ul style="list-style-type:none;">
     *     <li>0x00: Movement failed</li>
     *     <li>0x01: The motor is moving</li>
     *     <li>0x02: The motor has reached the target position</li>
     *     <li>0x03: An end limit has been reached</li>
     * </ul>
     *
     * @param speed          [uint16] target motor speed in units of 160/3 steps/s, see @ref Constants.MAX_SPEED for restrictions
     * @param acceleration   [uint8] motor acceleration, see @ref Constants.MAX_ACCEL for description
     * @param angle          [int24] angle to move by, in 1/2^14 of a rotation
     * @return status        [uint8] the status code described above
     */
    SEND_ANGLE = 0xF4,

    /** Run the motor to a specific angular position.
     * The manual refers to this as "Position mode4: absolute motion by axis [angle]".
     * Commands the motor to a particular position at a target speed with a fixed acceleration.
     * Angles are represented in the same fashion as @ref ENCODER_ADDITIVE, where a positive position indicates CW of
     *   zero, and negative CCW of zero.
     *
     * For example, an angular position of 0x4000 corresponds to 360° CW, and an angular position of 0xFFFF_FFFF_B000
     *   corresponds to 450° CCW.
     *
     * See @ref SEEK_POS_BY_STEPS for a variation of this command which takes the position as number of steps instead of
     *   an angle.
     *
     * Note this command behaves uniquely in that if another SEEK_POS_BY_ANGLE command is sent while this is executing,
     *   the first command will be dropped and the motor will seek the second position instead.
     * For other commands, the requests are queued so that the first command finishes before the second is executed.
     *
     * The motor can be stopped by sending another command with zero speed and angle.
     * If an acceleration of 0 is used, the motor will stop, or more generally reach the target speed, immediately.
     * Note that this may cause the motor to overshoot the original target if the acceleration is too slow.
     *
     * There are four legal return values:
     * <ul style="list-style-type:none;">
     *     <li>0x00: Movement failed</li>
     *     <li>0x01: The motor is moving</li>
     *     <li>0x02: The motor has reached the target position</li>
     *     <li>0x03: An end limit has been reached</li>
     * </ul>
     *
     * @param speed          [uint16] target motor speed in units of 160/3 steps/s, see @ref Constants.MAX_SPEED for restrictions
     * @param acceleration   [uint8] motor acceleration, see @ref Constants.MAX_ACCEL for description
     * @param steps          [int24] position to move to, in steps from the zero point
     * @return status        [uint8] the status code described above
     */
    SEEK_POS_BY_ANGLE = 0xF5,
};

/** Status code for the response to move commands.
 * Used by @ref SEND_STEP, @ref SEEK_POS_BY_STEPS, @ref SEND_ANGLE, and @ref SEEK_POS_BY_ANGLE.
 */
enum MksMoveResponse : uint8_t {
    /** Movement failed. */
    FAILED = 0,

    /** The motor is moving. */
    MOVING = 1,

    /** The motor has reached the target position. */
    COMPLETED = 2,

    /** An end limit has been reached. */
    LIMIT_REACHED = 3
};

#endif //UMRT_ARM_FIRMWARE_LIB_MKS_COMMANDS_HPP
