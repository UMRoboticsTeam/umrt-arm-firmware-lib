//
//  Created: Edcel 2026-04-12
//

/**
 *  Define the Rover Commands which will be sent to the STM32 
 */

#ifndef UMRT_ROVER_COMMANDS_HPP
#define UMRT_ROVER_COMMANDS_HPP

#include <stdint.h>

enum RoverCommands : uint8_t{

    /**
     *  Pings communication connection to STM32
     * 
     *  @param payload 
     *  @return 'payload'
    */
    STM_ECHO = 0x00,
    
    /**
     *  Sets the speed for the motors to be zero forcing a remote stop 
     * 
     *  @param speed [int16_t] - which will be 0 to force a remote stop 
     *  @return speed - which will be 0 
    */
    EMERGENCY_STOP = 0x01, 

    /**
     *  (might be removed, doesn't fit the scope of wheel_controller)
     *  Gets the status of the Rover: 
     *      - Temperature 
     *      - Battery Voltage
     *      - Current Consumption
     *      - etc.
     * 
     *  @param  
     *  @returns
     *      temperature
    */
    STATUS = 0x02, 

    /**
     *  Set's the speed of the motors 
     * 
     *  @param left_wheel_speed  [int16_t] - Angular Velocity of Left Motors
     *  @param right_wheel_speed [int16_t] - Angular Velocity of Right Motors 
     *  @return 'payload'
    */
    SET_SPEED = 0x03, 

    /**
     *  Return the current speed of the motors 
     * 
     *  @param right_wheel_speed [int16_t] - Angular Velocity of Right Motors 
     *  @param left_wheel_speed  [int16_t] - Angular Velocity of Left Motors
     *  @returns 
     *      left_wheel_speed
     *      right_wheel_speed
    */
    GET_SPEED = 0x04, 

    /**
     *  (might be removed, doesn't fit the scope of wheel_controller)
     *  Set's the servo motor's angle (0-270)
     * 
     *  @param yaw_angle [u_int16_t]  
     *  @return yaw_angle 
    */
    SET_CAMERA_SERVO = 0x05, 

    /**
     *  Respond back with payload, confirms communication connection to STM32
     * 
     *  @param payload 
     *  @return 'payload'
    */
    ECHO_RESPONSE = 0x06,

};  //  enum RoverCommands

#endif