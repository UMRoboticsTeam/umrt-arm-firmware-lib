//
// Created by Noah Reeder on 2024-05-30
// Based off of ArduinoTest from openFrameworksArduino (https://github.com/NeuroRoboticTech/openFrameworksArduino/blob/master/examples/ArduinoTest.cpp)
//

#include <string>

#include <boost/log/trivial.hpp>

#include "SYSEX_COMMANDS.h"
#include "StepperController.h"
#include "Utils.h"


StepperController::StepperController() : setup_completed(false) {
    BOOST_LOG_TRIVIAL(trace) << "StepperController construction begun";

    // Bind to the initialization connection of the ofArduino, and call this->setupArduino(majorFirmwareVersion)
    this->EInitialized.connect(boost::bind(&StepperController::setupArduino, this, _1));

    // Bind to the sysex received connection of the ofArduino, and call this->handleSysex(message)
    this->ESysExReceived.connect(boost::bind(&StepperController::handleSysex, this, _1));

    BOOST_LOG_TRIVIAL(debug) << "StepperController constructed";
}

StepperController::~StepperController() noexcept {
    BOOST_LOG_TRIVIAL(debug) << "StepperController destructed";
}

void StepperController::setupArduino(const int& version) {
    BOOST_LOG_TRIVIAL(trace) << "StepperController Arduino connection established";

    // For now, do nothing
    // May want to do things like setting step mode in the future

    this->setup_completed = true;

    BOOST_LOG_TRIVIAL(info) << "StepperController setup completed";
    this->ESetup();
}


bool StepperController::sendEcho(const std::vector<uint8_t>& payload) {
    if (!isSetup()) { return false; }

    sendSysEx(SysexCommands::ARDUINO_ECHO, payload);

    return true;
}


bool StepperController::setSpeed(const int16_t speed) {
    if (!isSetup()) { return false; }

    sendSysEx(SysexCommands::SET_SPEED, pack_16(speed));

    return true;
}


bool StepperController::getSpeed() {
    if (!isSetup()) { return false; }

    sendSysEx(SysexCommands::GET_SPEED, std::vector<uint8_t>());

    return true;
}

void StepperController::handleEArduinoEcho(const std::vector<unsigned char>& message) {
    BOOST_LOG_TRIVIAL(debug) << "ArduinoEcho received";
    this->EArduinoEcho(std::vector<uint8_t>(message.cbegin(), message.cend()));
}

void StepperController::handleESetSpeed(const std::vector<unsigned char>& message) {
    int16_t speed = static_cast<int16_t>(decode_16(message.cbegin()));
    BOOST_LOG_TRIVIAL(debug) << "SetSpeed received with speed=" << speed;
    this->ESetSpeed(speed);
}

void StepperController::handleEGetSpeed(const std::vector<unsigned char>& message) {
    int16_t speed = static_cast<int16_t>(decode_16(message.cbegin()));
    BOOST_LOG_TRIVIAL(debug) << "SetSpeed received with speed=" << speed;
    this->EGetSpeed(speed);
}

void StepperController::handleSysex(const std::vector<unsigned char>& message) {
    if (message.empty()) { // Must at least have command
        BOOST_LOG_TRIVIAL(error) << "SysEx received with no command byte";
        return;
    }

    if (!(message.size() % 2)) { // Must be odd since the first byte is the 7-bit command, followed by a firmatified payload
        BOOST_LOG_TRIVIAL(error) << "SysEx received with non-firmatified data";
        return;
    }

    // Defirmatify data - See firmatify_32 in Utils.h for explanation of why this is needed
    std::vector<unsigned char> defirmatified_message(message.size() / 2);
    for (int i = 0; i < message.size(); ++i) {
        defirmatified_message[i] = message[2*i + 1] | message[2*i + 2] << 7; // +1 since we don't want to include the command byte
    }

    // Process the message
    switch (message[0]) {
        case SysexCommands::ARDUINO_ECHO:
            this->handleEArduinoEcho(defirmatified_message);
            break;
        case SysexCommands::SET_SPEED:
            this->handleESetSpeed(defirmatified_message);
            break;
        case SysexCommands::GET_SPEED:
            this->handleEGetSpeed(defirmatified_message);
            break;
        default:
            BOOST_LOG_TRIVIAL(info) << "Unknown Sysex received with command=" << message[0];
            break;
    }
}