//
// Created by Noah on 2025-04-23.
//

#include <boost/log/trivial.hpp>

#include <ros2_socketcan/socket_can_receiver.hpp>
#include <ros2_socketcan/socket_can_sender.hpp>

#include <numeric>

#include "MKS_COMMANDS.hpp"
#include "mks_stepper_controller.hpp"
#include "utils.hpp"

uint8_t checksum(uint8_t driver_id, const std::vector<uint8_t>& payload);

MksStepperController::MksStepperController(const std::string& can_interface, const uint8_t norm_factor) : norm_factor(norm_factor),
                                                                                                          setup_completed(false) {
    BOOST_LOG_TRIVIAL(trace) << "ArduinoStepperController construction begun";

    this->can_receiver = std::make_unique<drivers::socketcan::SocketCanReceiver>(can_interface);
    this->can_sender = std::make_unique<drivers::socketcan::SocketCanSender>(can_interface);

    //TODO: Write norm_factor as microstepping factor to the driver

    BOOST_LOG_TRIVIAL(debug) << "MksStepperController constructed";
}

MksStepperController::~MksStepperController() noexcept {
    BOOST_LOG_TRIVIAL(debug) << "MksStepperController destructed";
}

bool MksStepperController::setSpeed(const uint8_t motor, const int16_t speed, const uint8_t acceleration) {
    if (!isSetup()) { return false; }

    // Speed is normalised when norm_factor is 16
    // That is, at 16 normalised_speed = speed
    // At 1, normalised_speed = speed / 16
    // At 32, normalised_speed = speed * 2
    int16_t normalised_speed = static_cast<int16_t>(speed * (int32_t)16 / norm_factor);

    std::vector<uint8_t> payload{ MksCommands::SET_SPEED };

    uint8_t speed_properties_low = static_cast<uint8_t>((normalised_speed & 0xF00) >> 8) | (normalised_speed < 0 ? 1u << 7 : 0u);
    uint8_t speed_properties_high = normalised_speed & 0xFF;
    payload.insert(payload.end(), speed_properties_low);
    payload.insert(payload.end(), speed_properties_high);
    payload.insert(payload.end(), acceleration);
    payload.insert(payload.end(), checksum(motor, payload));

    // TODO: Is the driver expecting remote messages?
    try {
        drivers::socketcan::CanId can_id(motor, 0, drivers::socketcan::FrameType::DATA, drivers::socketcan::StandardFrame);
        can_sender->send(payload.data(), payload.size(), can_id);
    } catch (drivers::socketcan::SocketCanTimeout& e) {
        // Won't bother with e.what(), it is always "CAN Send timeout"
        BOOST_LOG_TRIVIAL(warning) << "MksStepperController setSpeed timeout: motor=" << motor << ", speed=" << normalised_speed << ", accel=" << acceleration;
        return false;
    }
    return true;
}


bool MksStepperController::getSpeed(const uint8_t motor) {
    if (!isSetup()) { return false; }

    std::vector<uint8_t> payload{ MksCommands::MOTOR_SPEED };
    payload.insert(payload.end(), checksum(motor, payload));

    try {
        drivers::socketcan::CanId can_id(motor, 0, drivers::socketcan::FrameType::DATA, drivers::socketcan::StandardFrame);
        can_sender->send(payload.data(), payload.size(), can_id);
    } catch (drivers::socketcan::SocketCanTimeout& e) {
        BOOST_LOG_TRIVIAL(warning) << "MksStepperController getSpeed timeout: motor=" << motor;
        return false;
    }

    return true;
}

bool MksStepperController::sendStep(const uint8_t motor, const uint32_t num_steps, const int16_t speed, const uint8_t acceleration) {
    if (!isSetup()) { return false; }

    int16_t normalised_speed = static_cast<int16_t>(speed * (int32_t)16 / norm_factor);
    uint32_t normalised_steps = num_steps * norm_factor;

    std::vector<uint8_t> payload{ MksCommands::SEND_STEP };

    uint16_t abs_speed = std::abs<int16_t>(speed); // TODO: Don't use signed speed

    uint8_t speed_properties_low = static_cast<uint8_t>((normalised_speed & 0xF00) >> 8) | (normalised_speed < 0 ? 1u << 7 : 0u);
    uint8_t speed_properties_high = normalised_speed & 0xFF;
    auto steps_packed = pack_24_big(normalised_speed);
    payload.insert(payload.end(), speed_properties_low);
    payload.insert(payload.end(), speed_properties_high);
    payload.insert(payload.end(), acceleration);
    // With move iterators the compiler might invoke copy elision? Not entirely sure
    payload.insert(payload.end(), std::make_move_iterator(steps_packed.begin()), std::make_move_iterator(steps_packed.end()));
    payload.insert(payload.end(), checksum(motor, payload));

    try {
        drivers::socketcan::CanId can_id(motor, 0, drivers::socketcan::FrameType::DATA, drivers::socketcan::StandardFrame);
        can_sender->send(payload.data(), payload.size(), can_id);
    } catch (drivers::socketcan::SocketCanTimeout& e) {
        BOOST_LOG_TRIVIAL(warning) << "MksStepperController sendStep timeout: motor=" << motor << ", num_steps=" << num_steps << ", speed=" << normalised_speed << ", accel=" << acceleration;
        return false;
    }
    return true;
}

bool MksStepperController::seekPosition(const uint8_t motor, const int32_t position, const int16_t speed, const uint8_t acceleration) {
    if (!isSetup()) { return false; }

    int16_t normalised_speed = static_cast<int16_t>(speed * (int32_t)16 / norm_factor);
    int32_t normalised_position = position * norm_factor;

    std::vector<uint8_t> payload{ MksCommands::SEEK_POS_BY_STEPS };

    uint8_t speed_properties_low = static_cast<uint8_t>((normalised_speed & 0xF00) >> 8) | (normalised_speed < 0 ? 1u << 7 : 0u);
    uint8_t speed_properties_high = normalised_speed & 0xFF;
    auto steps_packed = pack_24_big(normalised_position);
    payload.insert(payload.end(), speed_properties_low);
    payload.insert(payload.end(), speed_properties_high);
    payload.insert(payload.end(), acceleration);
    // With move iterators the compiler might invoke copy elision? Not entirely sure
    payload.insert(payload.end(), std::make_move_iterator(steps_packed.begin()), std::make_move_iterator(steps_packed.end()));
    payload.insert(payload.end(), checksum(motor, payload));

    try {
        drivers::socketcan::CanId can_id(motor, 0, drivers::socketcan::FrameType::DATA, drivers::socketcan::StandardFrame);
        can_sender->send(payload.data(), payload.size(), can_id);
    } catch (drivers::socketcan::SocketCanTimeout& e) {
        BOOST_LOG_TRIVIAL(warning) << "MksStepperController sendStep timeout: motor=" << motor << ", position=" << normalised_position << ", speed=" << normalised_speed << ", accel=" << acceleration;
        return false;
    }
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

/**
 * Converts 16-bit unsigned integer to bytes in big-endian format.
 * @return big-endian representation
 */
std::vector<uint8_t> to_bytes(uint32_t integer) {
    return {
        static_cast<uint8_t>(integer >> 8 & 0xFF), // bits [15, 8]
        static_cast<uint8_t>(integer & 0xFF)       // bits [7, 0]
    };
}

uint8_t checksum(uint8_t driver_id, const std::vector<uint8_t>& payload) {
    // Note: Accumulate is going to work in uint8_t, and unsigned integer overflow is well-defined - no need for explicit modulo
    return std::accumulate(payload.cbegin(), payload.cend(), driver_id);
}