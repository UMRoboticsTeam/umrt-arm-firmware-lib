/**
* @file
* Defines the SysexCommands enum which lists the commands which are available to send a Stepper Controller over
* Firmata.
*/

#ifndef SYSEX_COMMANDS_H
#define SYSEX_COMMANDS_H

#include <cstdint>

/**
* Lists the command IDs and associated requirements which can be sent over Firmata to a Stepper Controller.
*/
// TODO: Consider adding a keep-alive
// Note: Only allowed to use 0x00-0x0F, beyond that would have to nest commands
// Multi-byte numbers are always transmitted in little-endian
enum SysexCommands : uint8_t {
    /**
    * Respond back with the payload.
    * @param payload a byte array of any length
    * @return `payload` exactly as it was received
    */
    ARDUINO_ECHO = 0x00,

    /**
    * Sets the speed of a motor.
    * @param motor_id [uint8_t] the ID of the motor to control
    * @param speed [int16_t] the signed speed to set the motor to, in 1/10 RPM
    * @returns
    *      [uint8_t] motor_id,<br>
    *      [int16_t] speed
    */
    SET_SPEED = 0x01,

    /**
    * Gets the current speed of a motor.
    * @param motor_id [uint8_t] the ID of the motor to query
    * @returns
    *      [uint8_t] motor_id,<br>
    *      [int16_t] speed
    */
    GET_SPEED = 0x02,

    /**
    * Moves a motor a specific number of steps. Direction is controlled by the sign of the speed.
    * @param motor_id [uint8_t] the ID of the motor to control
    * @param num_steps [uint16_t] the number of steps to move
    * @param speed [int16_t] the signed target speed to move the motor at, in 1/10 RPM
    * @returns
    *      [uint8_t] motor_id,<br>
    *      [uint16_t] num_steps,<br>
    *      [int16_t] speed
    */
    SEND_STEP = 0x03,

    /**
    * Sets the target position of the gripper servo.
    * The servo is controlled such that [0, 180] maps the entire servo range, meaning that a position of `180` may
    * correspond to 90°, 180°, 270°, etc. based on the particular servo attached. Values greater than `180` are
    * ignored.
    * @param position [uint8_t] the position of the servo, where [0, 180] maps the entire servo range
    * @returns
    *      [uint8_t] position
    */
    SET_GRIPPER = 0x04
};

#endif
