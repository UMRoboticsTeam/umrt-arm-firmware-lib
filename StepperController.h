//
// Created by Noah Reeder on 2024-05-30
// Based off of ArduinoTest from openFrameworksArduino (https://github.com/NeuroRoboticTech/openFrameworksArduino/blob/master/examples/ArduinoTest.h)
//

#ifndef ARM_FIRMWARE_LIB_STEPPERCONTROLLER_H
#define ARM_FIRMWARE_LIB_STEPPERCONTROLLER_H

#include <vector>

#include "StdAfx.h"
#include "ofArduino.h"
#include <boost/signals2.hpp>

/**
 * Manages the Firmata connection to an Arduino running the Stepper Controller program. Responses are conveyed through
 * <a href=https://www.boost.org/doc/libs/1_63_0/doc/html/signals.html>Boost signals</a>.
 */
class StepperController : public ofArduino {
public:
    /**
     * Initializes a StepperController.
     */
    StepperController();

    /**
     * Destroys a StepperController.
     */
    ~StepperController() noexcept override;

    /**
     * Sends a @ref SysexCommands::ARDUINO_ECHO command with the provided payload.
     * Response callbacks are available through @ref EArduinoEcho.
     *
     * @param payload byte vector to echo off the Stepper Controller Arduino
     * @return `true` if successfully written to the serial connection
     */
    bool sendEcho(const std::vector<uint8_t>& payload);

    /**
     * Sends a @ref SysexCommands::SET_SPEED command to set the speed of a motor.
     * Response callbacks are available through @ref ESetSpeed.
     *
     * @param motor the ID of the motor to control
     * @param speed the signed target speed to set the motor to, in 1/10 RPM
     * @return `true` if successfully written to the serial connection
     */
    bool setSpeed(const uint8_t motor, const int16_t speed);

    /**
     * Sends a @ref SysexCommands::GET_SPEED command to query the speed of a motor.
     * Response callbacks are available through @ref EGetSpeed.
     *
     * @param motor the ID of the motor to query
     * @return `true` if successfully written to the serial connection
     */
    // TODO: Would be nice to include timestamp in response since it is asynchronous
    bool getSpeed(const uint8_t motor);

    /**
     * Sends a @ref SysexCommands::SEND_STEP command to move a motor a fixed number of steps. Direction is controlled
     * by the sign of the target speed.
     * Response callbacks are available through @ref ESendStep.
     *
     * @param motor the ID of the motor to move
     * @param num_steps the number of steps to move
     * @param speed the signed target speed to move the motor at, in 1/10 RPM
     * @return `true` if successfully written to the serial connection
     */
    bool sendStep(const uint8_t motor, const uint16_t num_steps, const int16_t speed);

    /**
     * Sends a @ref SysexCommands::SET_GRIPPER command to set the target position of the gripper servo.
     * Response callbacks are available through @ref ESetGripper.
     *
     * @param position the target servo angle, mapped to [0, 180]
     * @return `true` if successfully written to the serial connection
     */
    bool setGripper(const uint8_t position);

    // Checks if this StepperController is fully setup
    /**
     * Returns whether the connection to the Stepper Controller Arduino has been fully established.
     * @return `true` if so
     */
    [[nodiscard]] bool isSetup() const { return this->setup_completed; };

    // ==========================
    //           Events
    // ==========================

    /**
     * <a href=https://www.boost.org/doc/libs/1_63_0/doc/html/signals.html>Boost signal</a> triggered once this
     * StepperController is fully setup.
     */
    boost::signals2::signal<void(void)> ESetup;

    /**
     * <a href=https://www.boost.org/doc/libs/1_63_0/doc/html/signals.html>Boost signal</a> triggered when
     * @ref sendEcho responses are received.
     */
    boost::signals2::signal<void(std::vector<uint8_t>)> EArduinoEcho;

    /**
     * <a href=https://www.boost.org/doc/libs/1_63_0/doc/html/signals.html>Boost signal</a> triggered when
     * @ref setSpeed responses are received.
     */
    boost::signals2::signal<void(uint8_t, int16_t)> ESetSpeed;

    /**
     * <a href=https://www.boost.org/doc/libs/1_63_0/doc/html/signals.html>Boost signal</a> triggered when
     * @ref getSpeed responses are received.
     */
    boost::signals2::signal<void(uint8_t, int16_t)> EGetSpeed;

    /**
     * <a href=https://www.boost.org/doc/libs/1_63_0/doc/html/signals.html>Boost signal</a> triggered when
     * @ref sendStep responses are received.
     */
    boost::signals2::signal<void(uint8_t, uint16_t, int16_t)> ESendStep;

    /**
     * <a href=https://www.boost.org/doc/libs/1_63_0/doc/html/signals.html>Boost signal</a> triggered when
     * @ref setGripper responses are received.
     */
    boost::signals2::signal<void(uint8_t)> ESetGripper;

protected:
    /**
     * <a href=https://www.boost.org/doc/libs/1_63_0/doc/html/signals.html>Boost signal</a> triggered when
     * the initial Firmata connection is established. Should immediately call @ref setupArduino.
     */
    boost::signals2::connection connectionInitialized;

    /**
     * Completes configuration of the Stepper Controller Arduino once a Firmata link has been established.
     *
     * @param version the Firmata version supplied with the @ref connectionInitialized response
     */
    void setupArduino(const int& version);

    // Note that using extended command IDs (i.e. command byte 0x00 followed by a 2 byte command ID) yields undefined behaviour
    // TODO: We therefore shouldn't be using 0x00 for ARDUINO_ECHO
    /**
     * Handles System-Exclusive (Sysex) messages received on the Firmata link with the Stepper Controller Arduino.
     *
     * @param message the message payload
     */
    void handleSysex(const std::vector<unsigned char>& message);

    /**
     * @name Signal Processing Helper Functions
     * Helper functions for decoding the parameters of Sysex commands processed by @ref handleSysex before forwarding
     * to their associated <a href=https://www.boost.org/doc/libs/1_63_0/doc/html/signals.html>signal</a>.
     *
     * @param message the de-firmatified Sysex payload
     */
    //@{
    void handleEArduinoEcho(const std::vector<unsigned char>& message);

    void handleESetSpeed(const std::vector<unsigned char>& message);

    void handleEGetSpeed(const std::vector<unsigned char>& message);

    void handleESendStep(const std::vector<unsigned char>& message);

    void handleESetGripper(const std::vector<unsigned char>& message);
    //@}

private:
    /**
     * Flag which indicates whether @ref setupArduino has completed configuring the Stepper Controller Arduino.
     */
    bool setup_completed;
};

#endif //ARM_FIRMWARE_LIB_STEPPERCONTROLLER_H
