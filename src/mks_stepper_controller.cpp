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
#include <cmath>

uint8_t checksum(uint16_t driver_id, const std::vector<uint8_t>& payload);

MksStepperController::MksStepperController(const std::string& can_interface, const uint8_t norm_factor) : norm_factor(norm_factor),
                                                                                                          setup_completed(false) {
    BOOST_LOG_TRIVIAL(trace) << "ArduinoStepperController construction begun";

    this->can_receiver = std::make_unique<drivers::socketcan::SocketCanReceiver>(can_interface);
    this->can_sender = std::make_unique<drivers::socketcan::SocketCanSender>(can_interface);

    //TODO: Write norm_factor as microstepping factor to the driver

    BOOST_LOG_TRIVIAL(debug) << "MksStepperController constructed";

    setup_completed = true;
}

MksStepperController::~MksStepperController() noexcept {
    BOOST_LOG_TRIVIAL(debug) << "MksStepperController destructed";
}

bool MksStepperController::setSpeed(const uint16_t motor, const int16_t speed, const uint8_t acceleration) {
    if (!isSetup()) { return false; }

    // Speed is normalised when norm_factor is 16
    // That is, at 16 normalised_speed = speed
    // At 1, normalised_speed = speed / 16
    // At 32, normalised_speed = speed * 2
    int16_t normalised_speed = static_cast<int16_t>(std::abs(speed) * (int32_t)16 / norm_factor);

    std::vector<uint8_t> payload{ MksCommands::SET_SPEED };

    uint8_t speed_properties_low = static_cast<uint8_t>((normalised_speed & 0xF00) >> 8) | (speed < 0 ? 1u << 7 : 0u);
    uint8_t speed_properties_high = static_cast<uint8_t>(normalised_speed & 0xFF);
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


bool MksStepperController::getSpeed(const uint16_t motor) {
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

bool MksStepperController::sendStep(const uint16_t motor, const uint32_t num_steps, const int16_t speed, const uint8_t acceleration) {
    if (!isSetup()) { return false; }

    // TODO: Don't use signed speed
    int16_t normalised_speed = static_cast<int16_t>(std::abs(speed) * (int32_t)16 / norm_factor);
    uint32_t normalised_steps = num_steps * norm_factor;

    std::vector<uint8_t> payload{ MksCommands::SEND_STEP };

    uint8_t speed_properties_low = static_cast<uint8_t>((normalised_speed & 0xF00) >> 8) | (speed < 0 ? 1u << 7 : 0u);
    uint8_t speed_properties_high = static_cast<uint8_t>(normalised_speed & 0xFF);
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

bool MksStepperController::seekPosition(const uint16_t motor, const int32_t position, const int16_t speed, const uint8_t acceleration) {
    if (!isSetup()) { return false; }

    int16_t normalised_speed = static_cast<int16_t>(std::abs(speed) * (int32_t)16 / norm_factor);
    int32_t normalised_position = position * norm_factor;

    std::vector<uint8_t> payload{ MksCommands::SEEK_POS_BY_STEPS };

    uint8_t speed_properties_low = static_cast<uint8_t>((normalised_speed & 0xF00) >> 8) | (speed < 0 ? 1u << 7 : 0u);
    uint8_t speed_properties_high = static_cast<uint8_t>(normalised_speed & 0xFF);
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

bool MksStepperController::getPosition(const uint16_t motor) {
    if (!isSetup()) { return false; }

    std::vector<uint8_t> payload{ MksCommands::CURRENT_POS };
    payload.insert(payload.end(), checksum(motor, payload));

    try {
        drivers::socketcan::CanId can_id(motor, 0, drivers::socketcan::FrameType::DATA, drivers::socketcan::StandardFrame);
        can_sender->send(payload.data(), payload.size(), can_id);
    } catch (drivers::socketcan::SocketCanTimeout& e) {
        BOOST_LOG_TRIVIAL(warning) << "MksStepperController getPosition timeout: motor=" << motor;
        return false;
    }
    return true;
}

bool MksStepperController::isSetup() const { return this->setup_completed; };

void MksStepperController::update(const std::chrono::nanoseconds & timeout) {
    // Read a message from the CAN bus
    // TODO: Consider bus-level message filtering for efficiency
    // TODO: Need to at least do driver filtering, otherwise could get messages which happen to have the same 1st byte
    //       as one of our commands, and then attempt to unsafely decode something
    uint8_t msg_buffer[8];
    drivers::socketcan::CanId msg_info = this->can_receiver->receive(&msg_buffer, timeout);

    // If this isn't a standard CAN message, then it isn't a message applicable to us
    if (msg_info.frame_type() != drivers::socketcan::FrameType::DATA) { return; }

    // Turn the raw buffer into a vector
    std::vector msg(msg_buffer, msg_buffer + msg_info.length());

    this->handleCanMessage(msg, msg_info);
}


void MksStepperController::handleESetSpeed(const std::vector<unsigned char>& message) {
    // TODO: Decode response
}

void MksStepperController::handleEGetSpeed(const std::vector<unsigned char>& message) {
    // TODO: Decode response
}

void MksStepperController::handleESendStep(const std::vector<unsigned char>& message) {
    // TODO: Decode response
}

void MksStepperController::handleESeekPosition(const std::vector<unsigned char>& message) {
    // TODO: Decode response
}

void MksStepperController::handleEGetPosition(const std::vector<uint8_t>& message, drivers::socketcan::CanId & info) {
    auto position = static_cast<int32_t>(decode_32_big(message.cbegin() + 1));
    BOOST_LOG_TRIVIAL(debug) << "[" << info.get_bus_time() << "]: GetPosition received for motor " << info.identifier()
                             << " with position=" << position << ", normalised_position=" << position / norm_factor;
    EGetPosition(info.identifier(), position / norm_factor);
}

void MksStepperController::handleCanMessage(const std::vector<uint8_t>& message, drivers::socketcan::CanId & info) {
    // Note: info can't be const because get_bus_time isn't const-qualified...

    if (message.empty()) { // Must at least have command for us to process
        // We are subscribing to all messages on the bus, if another device is using payload-less messages there is no
        // reason to spam our log over it
        return;
    }

    // Process the message
    switch (message[0]) {
        case MksCommands::CURRENT_POS: this->handleEGetPosition(message, info); break;
        default:
            // Again, we are subscribing to all messages on the bus, no need to spam log with ignored messages
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

uint8_t checksum(uint16_t driver_id, const std::vector<uint8_t>& payload) {
    // Note: Accumulate is going to work in uint8_t, and unsigned integer overflow is well-defined - no need for explicit modulo
    return std::accumulate(payload.cbegin(), payload.cend(), static_cast<uint8_t>(driver_id));
}