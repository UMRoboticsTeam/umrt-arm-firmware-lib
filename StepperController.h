//
// Created by Noah Reeder on 2024-05-30
// Based off of ArduinoTest from openFrameworksArduino (https://github.com/NeuroRoboticTech/openFrameworksArduino/blob/master/examples/ArduinoTest.h)
//

#ifndef COMMUNICATION_MASTER_EXAMPLE_STEPPERCONTROLLER_H
#define COMMUNICATION_MASTER_EXAMPLE_STEPPERCONTROLLER_H

#include <vector>

#include "StdAfx.h"
#include "ofArduino.h"
#include <boost/signals2.hpp>

class StepperController : public ofArduino {
public:
    StepperController();

    ~StepperController() noexcept override;

    bool sendEcho(const std::vector<uint8_t>& payload);

    bool setSpeed(const int16_t speed);

    bool getSpeed();

    bool sendStep(const uint16_t num_steps, const int16_t speed);

    // Checks if this StepperController is fully setup
    bool isSetup() const { return this->setup_completed; };

    // ==========================
    //           Events
    // ==========================

    // Triggered once this StepperController is fully setup
    boost::signals2::signal<void(void)> ESetup;

    // Triggered when echo responses are received
    boost::signals2::signal<void(std::vector<uint8_t>)> EArduinoEcho;

    // Triggered when setSpeed responses are received
    boost::signals2::signal<void(int16_t)> ESetSpeed;

    // Triggered when getSpeed responses are received
    boost::signals2::signal<void(int16_t)> EGetSpeed;

    // Triggered when sendStep responses are received
    boost::signals2::signal<void(uint16_t, int16_t)> ESendStep;

protected:
    boost::signals2::connection connectionInitialized;

    void setupArduino(const int& version);

    // Note that using extended command IDs (i.e. command byte 0x00 followed by a 2 byte command ID) yields undefined behaviour
    // TODO: We therefore shouldn't be using 0x00 for ARDUINO_ECHO
    void handleSysex(const std::vector<unsigned char>& message);

    void handleEArduinoEcho(const std::vector<unsigned char>& message);

    void handleESetSpeed(const std::vector<unsigned char>& message);

    void handleEGetSpeed(const std::vector<unsigned char>& message);

    void handleESendStep(const std::vector<unsigned char>& message);

private:
    bool setup_completed;
};


#endif //COMMUNICATION_MASTER_EXAMPLE_STEPPERCONTROLLER_H
