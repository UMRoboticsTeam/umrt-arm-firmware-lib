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

/**
 * PIMPL wrapper around the CAN send/receive implementation to allow for it to be a private dependency.
 * Under no circumstance may an instance of this class outlive the MksStepperController which encloses it.
 */
class MksStepperController::CanImpl {
public:
    explicit CanImpl(MksStepperController* controller, const std::string& can_interface) : controller(controller) {
        this->can_receiver = std::make_unique<drivers::socketcan::SocketCanReceiver>(can_interface);
        this->can_sender = std::make_unique<drivers::socketcan::SocketCanSender>(can_interface);
    }

    /**
     * Handles received CAN messages and sends out signals as appropriate.
     *
     * @param message the message payload
     * @param info auxiliary information associated with the message, e.g. driver ID, bus time
     */
    void handleCanMessage(const std::vector<uint8_t>& message, drivers::socketcan::CanId& info);

    /**
     * @name Signal Processing Helper Functions
     * Helper functions for decoding the parameters of Sysex commands processed by @ref handleSysex before forwarding
     * to their associated <a href=https://www.boost.org/doc/libs/1_63_0/doc/html/signals.html>signal</a>.
     *
     * @param message the de-firmatified Sysex payload
     */
    //@{
    void handleESetSpeed(const std::vector<uint8_t>& message, drivers::socketcan::CanId& info);

    void handleESendStep(const std::vector<uint8_t>& message, drivers::socketcan::CanId& info);

    void handleESeekPosition(const std::vector<uint8_t>& message, drivers::socketcan::CanId& info);

    void handleEGetPosition(const std::vector<unsigned char>& message, drivers::socketcan::CanId& info);
    //@}

    /**
     * Non-owning reference to the MksStepperController this CanImpl belongs to.
     */
    MksStepperController* controller;

    /**
     * Implementation-opaque SocketCAN receiver node.
     */
    std::unique_ptr<drivers::socketcan::SocketCanReceiver> can_receiver;

    /**
     * Implementation-opaque SocketCAN sender node.
     */
    std::unique_ptr<drivers::socketcan::SocketCanSender> can_sender;
};


MksStepperController::MksStepperController(
        const std::string& can_interface, std::shared_ptr<const std::unordered_set<uint16_t>> motor_ids,
        const uint8_t norm_factor
) : motor_ids{ std::move(motor_ids) }, norm_factor{ norm_factor } {
    BOOST_LOG_TRIVIAL(trace) << "MksStepperController construction begun";

    can_impl = std::make_unique<CanImpl>(this, can_interface);

    //TODO: Write norm_factor as microstepping factor to the driver

    BOOST_LOG_TRIVIAL(debug) << "MksStepperController constructed";

    setup_completed = true;
}

MksStepperController::~MksStepperController() noexcept { BOOST_LOG_TRIVIAL(debug) << "MksStepperController destructed"; }

bool MksStepperController::setSpeed(const uint16_t motor, const int16_t speed, const uint8_t acceleration) {
    if (!isSetup()) { return false; }

    // Speed is normalised when norm_factor is 16
    // That is, at 16 normalised_speed = speed
    // At 1, normalised_speed = speed / 16
    // At 32, normalised_speed = speed * 2
    auto normalised_speed = static_cast<int16_t>(std::abs(speed) * (int32_t)16 / norm_factor);

    std::vector<uint8_t> payload{ MksCommands::SET_SPEED };

    auto speed_properties_low = static_cast<uint8_t>((normalised_speed & 0xF00) >> 8 | (speed < 0 ? 1u << 7 : 0u));
    auto speed_properties_high = static_cast<uint8_t>(normalised_speed & 0xFF);
    payload.insert(payload.end(), speed_properties_low);
    payload.insert(payload.end(), speed_properties_high);
    payload.insert(payload.end(), acceleration);
    payload.insert(payload.end(), checksum(motor, payload));

    try {
        drivers::socketcan::CanId can_id(motor, 0, drivers::socketcan::FrameType::DATA, drivers::socketcan::StandardFrame);
        can_impl->can_sender->send(payload.data(), payload.size(), can_id);
    } catch (drivers::socketcan::SocketCanTimeout& e) {
        // Won't bother with e.what(), it is always "CAN Send timeout"
        BOOST_LOG_TRIVIAL(warning) << "MksStepperController setSpeed timeout: motor=0x" << std::hex << motor << std::dec
                                   << ", speed=" << normalised_speed << ", accel=" << acceleration;
        return false;
    }
    return true;
}

// Included for posterity; due to strange responses this command is assumed to return the encoder speed, and we are not
// using the driver's encoder so I cannot test this and it is not useful functionality anyways
//bool MksStepperController::getSpeed(const uint16_t motor) {
//    if (!isSetup()) { return false; }
//
//    std::vector<uint8_t> payload{ MksCommands::MOTOR_SPEED };
//    payload.insert(payload.end(), checksum(motor, payload));
//
//    try {
//        drivers::socketcan::CanId can_id(motor, 0, drivers::socketcan::FrameType::DATA, drivers::socketcan::StandardFrame);
//        can_impl->can_sender->send(payload.data(), payload.size(), can_id);
//    } catch (drivers::socketcan::SocketCanTimeout& e) {
//        BOOST_LOG_TRIVIAL(warning) << "MksStepperController getSpeed timeout: motor=" << motor;
//        return false;
//    }
//
//    return true;
//}

bool MksStepperController::sendStep(
        const uint16_t motor, const uint32_t num_steps, const int16_t speed, const uint8_t acceleration
) {
    if (!isSetup()) { return false; }

    auto normalised_speed = static_cast<int16_t>(std::abs(speed) * (int32_t)16 / norm_factor);
    uint32_t normalised_steps = num_steps * norm_factor;

    std::vector<uint8_t> payload{ MksCommands::SEND_STEP };

    auto speed_properties_low = static_cast<uint8_t>((normalised_speed & 0xF00) >> 8 | (speed < 0 ? 1u << 7 : 0u));
    auto speed_properties_high = static_cast<uint8_t>(normalised_speed & 0xFF);
    auto steps_packed = pack_24_big(normalised_steps);
    payload.insert(payload.end(), speed_properties_low);
    payload.insert(payload.end(), speed_properties_high);
    payload.insert(payload.end(), acceleration);
    // With move iterators the compiler might invoke copy elision? Not entirely sure
    payload.insert(
            payload.end(), std::make_move_iterator(steps_packed.begin()), std::make_move_iterator(steps_packed.end())
    );
    payload.insert(payload.end(), checksum(motor, payload));

    try {
        drivers::socketcan::CanId can_id(motor, 0, drivers::socketcan::FrameType::DATA, drivers::socketcan::StandardFrame);
        can_impl->can_sender->send(payload.data(), payload.size(), can_id);
    } catch (drivers::socketcan::SocketCanTimeout& e) {
        BOOST_LOG_TRIVIAL(warning) << "MksStepperController sendStep timeout: motor=0x" << std::hex << motor << std::dec
                                   << ", num_steps=" << num_steps << ", speed=" << normalised_speed
                                   << ", accel=" << acceleration;
        return false;
    }
    return true;
}

bool MksStepperController::seekPosition(
        const uint16_t motor, const int32_t position, const int16_t speed, const uint8_t acceleration
) {
    if (!isSetup()) { return false; }

    auto normalised_speed = static_cast<int16_t>(std::abs(speed) * (int32_t)16 / norm_factor);
    int32_t normalised_position = position * norm_factor;

    std::vector<uint8_t> payload{ MksCommands::SEEK_POS_BY_STEPS };

    auto speed_properties_low = static_cast<uint8_t>((normalised_speed & 0xF00) >> 8 | (speed < 0 ? 1u << 7 : 0u));
    auto speed_properties_high = static_cast<uint8_t>(normalised_speed & 0xFF);
    auto steps_packed = pack_24_big(normalised_position);
    payload.insert(payload.end(), speed_properties_low);
    payload.insert(payload.end(), speed_properties_high);
    payload.insert(payload.end(), acceleration);
    // With move iterators the compiler might invoke copy elision? Not entirely sure
    payload.insert(
            payload.end(), std::make_move_iterator(steps_packed.begin()), std::make_move_iterator(steps_packed.end())
    );
    payload.insert(payload.end(), checksum(motor, payload));

    try {
        drivers::socketcan::CanId can_id(motor, 0, drivers::socketcan::FrameType::DATA, drivers::socketcan::StandardFrame);
        can_impl->can_sender->send(payload.data(), payload.size(), can_id);
    } catch (drivers::socketcan::SocketCanTimeout& e) {
        BOOST_LOG_TRIVIAL(warning) << "MksStepperController sendStep timeout: motor=0x" << std::hex << motor << std::dec
                                   << ", position=" << normalised_position << ", speed=" << normalised_speed
                                   << ", accel=" << acceleration;
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
        can_impl->can_sender->send(payload.data(), payload.size(), can_id);
    } catch (drivers::socketcan::SocketCanTimeout& e) {
        BOOST_LOG_TRIVIAL(warning) << "MksStepperController getPosition timeout: motor=0x" << std::hex << motor << std::dec;
        return false;
    }
    return true;
}

bool MksStepperController::isSetup() const { return this->setup_completed; }

void MksStepperController::update(const std::chrono::nanoseconds& timeout) {
    // Read a message from the CAN bus
    // TODO: Consider bus-level message filtering for efficiency
    uint8_t msg_buffer[8];
    drivers::socketcan::CanId msg_info = can_impl->can_receiver->receive(&msg_buffer, timeout);

    // If this isn't a standard CAN message, then it isn't a message applicable to us
    if (msg_info.frame_type() != drivers::socketcan::FrameType::DATA) { return; }

    // Turn the raw buffer into a vector
    std::vector msg(msg_buffer, msg_buffer + msg_info.length());

    this->can_impl->handleCanMessage(msg, msg_info);
}


void MksStepperController::CanImpl::handleESetSpeed(const std::vector<uint8_t>& message, drivers::socketcan::CanId& info) {
    if (message.size() != 3) { return; } // Don't want to process loop-backed requests, only responses
    const auto status = static_cast<MksMoveResponse>(message.at(1));
    BOOST_LOG_TRIVIAL(debug) << "[" << info.get_bus_time() << "]: SetSpeed received for motor 0x" << std::hex
                             << info.identifier() << std::dec << " with status=" << status;
    controller->ESetSpeed(static_cast<uint16_t>(info.identifier()), status == 1);
}

void MksStepperController::CanImpl::handleESendStep(const std::vector<uint8_t>& message, drivers::socketcan::CanId& info) {
    if (message.size() != 3) { return; } // Don't want to process loop-backed requests, only responses
    const auto status = static_cast<MksMoveResponse>(message.at(1));
    BOOST_LOG_TRIVIAL(debug) << "[" << info.get_bus_time() << "]: SendStep received for motor 0x" << std::hex
                             << info.identifier() << std::dec << " with status=" << status;
    controller->ESendStep(static_cast<uint16_t>(info.identifier()), status);
}

void MksStepperController::CanImpl::handleESeekPosition(const std::vector<uint8_t>& message, drivers::socketcan::CanId& info) {
    if (message.size() != 3) { return; } // Don't want to process loop-backed requests, only responses
    const auto status = static_cast<MksMoveResponse>(message.at(1));
    BOOST_LOG_TRIVIAL(debug) << "[" << info.get_bus_time() << "]: SeekPosition received for motor 0x" << std::hex
                             << info.identifier() << std::dec << " with status=" << status;
    controller->ESeekPosition(static_cast<uint16_t>(info.identifier()), status);
}

void MksStepperController::CanImpl::handleEGetPosition(const std::vector<uint8_t>& message, drivers::socketcan::CanId& info) {
    if (message.size() != 6) { return; } // Don't want to process loop-backed requests, only responses
    auto position = static_cast<int32_t>(decode_32_big(message.cbegin() + 1));
    BOOST_LOG_TRIVIAL(debug) << "[" << info.get_bus_time() << "]: GetPosition received for motor 0x" << std::hex
                             << info.identifier() << std::dec << " with position=" << position
                             << ", normalised_position=" << position / controller->norm_factor;
    controller->EGetPosition(static_cast<uint16_t>(info.identifier()), position / controller->norm_factor);
}

void MksStepperController::CanImpl::handleCanMessage(const std::vector<uint8_t>& message, drivers::socketcan::CanId& info) {
    // Note: info can't be const because get_bus_time isn't const-qualified...

    // Drop message if not addressed to us
    BOOST_LOG_TRIVIAL(error) << "Reached" << info.is_extended();
    if (info.is_extended() || !controller->motor_ids->count(static_cast<uint16_t>(info.identifier()))) {
        // We are subscribing to all messages on the bus, there is no reason to spam our log over it
        return;
    }

    // So this message is from a motor driver; it must contain a command or there is something weird happening
    if (message.empty()) {
        BOOST_LOG_TRIVIAL(error) << "[" << info.get_bus_time() << "]: Message received for motor 0x" << std::hex
                                 << info.identifier() << std::dec << " with no payload";
    }

    // Process the message
    switch (message[0]) {
        case MksCommands::SET_SPEED: this->handleESetSpeed(message, info); break;
        case MksCommands::SEND_STEP: this->handleESendStep(message, info); break;
        case MksCommands::SEEK_POS_BY_STEPS: this->handleESeekPosition(message, info); break;
        case MksCommands::CURRENT_POS: this->handleEGetPosition(message, info); break;
        default:
            // Again, we are subscribing to all messages on the bus, no need to spam log with ignored messages
            break;
    }
}

/**
 * Calculates the "CRC" for an MKS message in accordance with @ref MksCommands
 * @param driver_id CAN ID of the driver this message is sent to
 * @param payload CAN message payload
 * @return computed checksum for the CAN message
 */
uint8_t checksum(uint16_t driver_id, const std::vector<uint8_t>& payload) {
    // Note: Accumulate is going to work in uint8_t, and unsigned integer overflow is well-defined - no need for explicit modulo
    return std::accumulate(payload.cbegin(), payload.cend(), static_cast<uint8_t>(driver_id));
}