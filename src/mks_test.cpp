//
// Created by Noah on 2025-04-22.
//

#include "mks_test.hpp"

#include <iostream>
#include <string>
#include <unordered_set>
#include <utils.hpp>

#include <ros2_socketcan/socket_can_id.hpp>

MksTest::MksTest(const std::string& can_interface, std::vector<uint16_t>&& motor_ids, const uint8_t norm_factor)
    : s{ can_interface, std::make_shared<std::unordered_set<uint16_t>>(motor_ids.cbegin(), motor_ids.cend()), norm_factor },
      motor_ids{ std::move(motor_ids) } {
    // Note: We use vector for motor_ids here instead of unordered_set because we want the motors to be tested in order
    s.ESetSpeed.connect([this](auto motor, auto status) { onSetSpeed(motor, status); });
    s.ESendStep.connect([this](auto motor, auto status) { onSendStep(motor, status); });
    s.ESeekPosition.connect([this](auto motor, auto status) { onSeekPosition(motor, status); });
    s.EGetPosition.connect([this](auto motor, auto position) { onGetPosition(motor, position); });

    std::cout << "Mks setup!" << std::endl;

    // Start the test procedure
    test_thread = std::thread(&MksTest::sendTestRoutine, this);
}

void MksTest::update() {
    try {
        s.update(std::chrono::nanoseconds(10));
    } catch (drivers::socketcan::SocketCanTimeout& _) {}
}

void MksTest::sendTestRoutine() {
    // Wait 1 second
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Test motors
    for (uint16_t motor : this->motor_ids) {
        // Send speed of 2 RPM for 5 seconds, then 1 RPM in other direction for 5 seconds, then stop
        s.getPosition(motor);
        s.setSpeed(motor, 2);
        std::this_thread::sleep_for(std::chrono::seconds(5));
        s.setSpeed(motor, -1);
        std::this_thread::sleep_for(std::chrono::seconds(5));
        s.setSpeed(motor, 0);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        //Step forward 20 steps at 10 RPM, then back 10 steps at 5 RPM
        s.getPosition(motor);
        s.sendStep(motor, 20, 10);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        s.getPosition(motor);
        s.sendStep(motor, 10, -5);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        s.getPosition(motor);

        // Wait 1 second
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Seek back to position -10 from wherever we ended up at 30 RPM
        s.seekPosition(motor, -10, 30);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        s.getPosition(motor);

        // Wait 1 second
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Seek back to position 0 from wherever we ended up at 10 RPM
        s.seekPosition(motor, 0, 10);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        s.getPosition(motor);

        // Wait 1 second
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void MksTest::onSetSpeed(const uint16_t motor, const bool status) {
    std::cout << "(Requested) Motor 0x" << std::hex << motor << std::dec
              << ": SetSpeed: success=" << (status ? "true" : "false") << std::endl;
}

void MksTest::onSendStep(const uint16_t motor, const MksMoveResponse status) {
    std::cout << "(Requested) Motor 0x" << std::hex << motor << std::dec
              << ": SendStep: status=" << to_string_mks_move_response(status) << std::endl;
}

void MksTest::onSeekPosition(const uint16_t motor, const MksMoveResponse status) {
    std::cout << "(Requested) Motor 0x" << std::hex << motor << std::dec
              << ": SeekPos: status=" << to_string_mks_move_response(status) << std::endl;
}

void MksTest::onGetPosition(const uint16_t motor, const int32_t position) {
    std::cout << "(Queried)   Motor 0x" << std::hex << motor << std::dec << ": GetPos: position=" << position << std::endl;
}
