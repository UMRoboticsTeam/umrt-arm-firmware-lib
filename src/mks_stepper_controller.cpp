//
// Created by Noah on 2025-04-23.
//

#include <boost/log/trivial.hpp>

#include "mks_stepper_controller.hpp"

MksStepperController::MksStepperController() : setup_completed(false) {
    BOOST_LOG_TRIVIAL(trace) << "ArduinoStepperController construction begun";

    // Bind to the initialization connection of the ofArduino, and call this->setupArduino(majorFirmwareVersion)
    this->EInitialized.connect(boost::bind(&ArduinoStepperController::setupArduino, this, _1));

    // Bind to the sysex received connection of the ofArduino, and call this->handleSysex(message)
    this->ESysExReceived.connect(boost::bind(&ArduinoStepperController::handleSysex, this, _1));

    BOOST_LOG_TRIVIAL(debug) << "MksStepperController constructed";
}

MksStepperController::~MksStepperController() noexcept {
    BOOST_LOG_TRIVIAL(debug) << "MksStepperController destructed";
}

bool MksStepperController::setSpeed(const uint8_t motor, const int16_t speed, const uint8_t acceleration) {
    if (!isSetup()) { return false; }

    std::vector<uint8_t> pack = { motor };
    auto speed_packed = pack_16(speed);
    pack.insert(pack.end(), speed_packed.cbegin(), speed_packed.cend());
    sendSysEx(SysexCommands::SET_SPEED, pack);

    return true;
}


bool MksStepperController::getSpeed(const uint8_t motor) {
    if (!isSetup()) { return false; }

    sendSysEx(SysexCommands::GET_SPEED, std::vector<uint8_t>({ motor }));

    return true;
}

bool MksStepperController::sendStep(const uint8_t motor, const uint16_t num_steps, const int16_t speed, const uint8_t acceleration) {
    if (!isSetup()) { return false; }

    std::vector<uint8_t> pack = { motor };
    auto steps_packed = pack_16(num_steps);
    auto speed_packed = pack_16(speed);
    pack.insert(pack.end(), steps_packed.cbegin(), steps_packed.cend());
    pack.insert(pack.end(), speed_packed.cbegin(), speed_packed.cend());
    sendSysEx(SysexCommands::SEND_STEP, pack);

    return true;
}

bool MksStepperController::seekPosition(const uint8_t motor, const int32_t position, const int16_t speed, const uint8_t acceleration) {
    if (!isSetup()) { return false; }

    std::vector<uint8_t> pack = { motor };
    auto position_packed = pack_32(position);
    auto speed_packed = pack_16(speed);
    pack.insert(pack.end(), position_packed.cbegin(), position_packed.cend());
    pack.insert(pack.end(), speed_packed.cbegin(), speed_packed.cend());
    sendSysEx(SysexCommands::SEEK_POS, pack);

    return true;
}

bool MksStepperController::getPosition(const uint8_t motor) {
    if (!isSetup()) { return false; }

    sendSysEx(SysexCommands::GET_POS, std::vector<uint8_t>({ motor }));

    return true;
}

bool MksStepperController::isSetup() const { return this->setup_completed; };


void MksStepperController::handleESetSpeed(const std::vector<unsigned char>& message) {
    auto it = message.cbegin();
    uint8_t motor = *it;
    it += 1;
    auto speed = static_cast<int16_t>(decode_16(it));
    BOOST_LOG_TRIVIAL(debug) << "SetSpeed received for motor " << motor << " with speed=" << speed;
    this->ESetSpeed(motor, speed);
}

void MksStepperController::handleEGetSpeed(const std::vector<unsigned char>& message) {
    auto it = message.cbegin();
    uint8_t motor = *it;
    it += 1;
    auto speed = static_cast<int16_t>(decode_16(it));
    BOOST_LOG_TRIVIAL(debug) << "GetSpeed received for motor " << motor << " with speed=" << speed;
    this->EGetSpeed(motor, speed);
}

void MksStepperController::handleESendStep(const std::vector<unsigned char>& message) {
    auto it = message.cbegin();
    uint8_t motor = *it;
    it += 1;
    auto steps = static_cast<uint16_t>(decode_16(it));
    it += 2;
    auto speed = static_cast<int16_t>(decode_16(it));
    BOOST_LOG_TRIVIAL(debug) << "SendStep received for motor " << motor << " with steps=" << steps << ", speed="
                             << speed;
    this->ESendStep(motor, steps, speed);
}

void MksStepperController::handleESeekPosition(const std::vector<unsigned char>& message) {
    auto it = message.cbegin();
    uint8_t motor = *it;
    it += 1;
    auto position = static_cast<int32_t>(decode_32(it));
    it += 4;
    auto speed = static_cast<int16_t>(decode_16(it));
    BOOST_LOG_TRIVIAL(debug) << "SeekPosition received for motor " << motor << " with position=" << position << ", speed="
                             << speed;
    this->ESeekPosition(motor, position, speed);
}

void MksStepperController::handleEGetPosition(const std::vector<unsigned char>& message) {
    auto it = message.cbegin();
    uint8_t motor = *it;
    it += 1;
    auto position = static_cast<int32_t>(decode_32(it));
    BOOST_LOG_TRIVIAL(debug) << "GetPosition received for motor " << motor << " with position=" << position;
    this->EGetPosition(motor, position);
}

void MksStepperController::handleCanMessage(const std::vector<unsigned char>& message) {
    if (message.empty()) { // Must at least have command
        BOOST_LOG_TRIVIAL(error) << "SysEx received with no command byte";
        return;
    }

    // Must be odd since the first byte is the 7-bit command, followed by a firmatified payload
    if (!(message.size() % 2)) {
        BOOST_LOG_TRIVIAL(error) << "SysEx received with non-firmatified data";
        return;
    }

    // Defirmatify data - See firmatify_32 in Utils.h for explanation of why this is needed
    std::vector<unsigned char> defirmatified_message(message.size() / 2);
    for (int i = 0; i < message.size(); ++i) {
        // +1 since we don't want to include the command byte
        defirmatified_message[i] = message[2 * i + 1] | message[2 * i + 2] << 7;
    }

    // Process the message
    switch (message[0]) {
        case SysexCommands::ARDUINO_ECHO: this->handleEArduinoEcho(defirmatified_message); break;
        case SysexCommands::SET_SPEED: this->handleESetSpeed(defirmatified_message); break;
        case SysexCommands::GET_SPEED: this->handleEGetSpeed(defirmatified_message); break;
        case SysexCommands::SEND_STEP: this->handleESendStep(defirmatified_message); break;
        case SysexCommands::SEEK_POS: this->handleESeekPosition(defirmatified_message); break;
        case SysexCommands::GET_POS: this->handleEGetPosition(defirmatified_message); break;
        case SysexCommands::SET_GRIPPER: this->handleESetGripper(defirmatified_message); break;
        default:
            BOOST_LOG_TRIVIAL(info) << "Unknown Sysex received with command=" << message[0];
            break;
    }
}