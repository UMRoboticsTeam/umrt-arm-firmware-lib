//
// Created by Noah on 2025-07-30.
//

#include "servo_controller.hpp"

#include <boost/log/trivial.hpp>
#include <ros2_socketcan/socket_can_sender.hpp>

ServoController::ServoController(const std::string& can_interface, const uint16_t servo_id) : servo_id_{ servo_id } {
    BOOST_LOG_TRIVIAL(trace) << "ServoController construction begun";

    can_sender_ = std::make_unique<drivers::socketcan::SocketCanSender>(can_interface);

    BOOST_LOG_TRIVIAL(debug) << "ServoController constructed";

    setup_completed_ = true;
}

ServoController::~ServoController() noexcept { BOOST_LOG_TRIVIAL(debug) << "ServoController destructed"; }

bool ServoController::send(const uint8_t position) {
    if (!isSetup()) { return false; }

    // Message format is an 8 byte payload where the 1st byte is the commanded position of the servo
    std::vector<uint8_t> payload{ position, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    try {
        drivers::socketcan::CanId can_id(
                servo_id_, 0, drivers::socketcan::FrameType::DATA, drivers::socketcan::ExtendedFrame
        );
        can_sender_->send(payload.data(), payload.size(), can_id);
    } catch (drivers::socketcan::SocketCanTimeout& e) {
        // Won't bother with e.what(), it is always "CAN Send timeout"
        BOOST_LOG_TRIVIAL(warning) << "ServoController send timeout: servo_id=" << servo_id_ << ", pos=" << position;
        return false;
    }
    return true;
}

bool ServoController::isSetup() const { return setup_completed_; }
