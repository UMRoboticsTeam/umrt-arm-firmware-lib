//
// Created by Noah on 2025-04-22.
//

#include "mks_test.hpp"

#include <iostream>
#include <ros2_socketcan/socket_can_id.hpp>

MksTest::MksTest(const std::string& can_interface, const std::vector<uint8_t>& motor_ids, const uint8_t norm_factor) : motor_ids(motor_ids),
                                                                                            s(can_interface, norm_factor) {
    /*s.ESetSpeed.connect([this](auto&& PH1, auto&& PH2) {
        onSetSpeed(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
    });
    s.EGetSpeed.connect([this](auto&& PH1, auto&& PH2) {
        onGetSpeed(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
    });
    s.ESendStep.connect([this](auto&& PH1, auto&& PH2, auto&& PH3) {
        onSendStep(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2), std::forward<decltype(PH3)>(PH3));
    });
    s.ESeekPosition.connect([this](auto&& PH1, auto&& PH2, auto&& PH3) {
        onSeekPosition(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2), std::forward<decltype(PH3)>(PH3));
    });*/
    s.EGetPosition.connect([this](auto motor, auto position) {
        onGetPosition(motor, position);
    });

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
    for (uint8_t motor : this->motor_ids) {
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

        // Seek back to position 0 from wherever we ended up at 10 RPM
        s.seekPosition(motor, 0, 10);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        s.getPosition(motor);

        // Wait 1 second
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void MksTest::onSetSpeed(const uint16_t motor, const int16_t speed) {}

void MksTest::onSendStep(const uint16_t motor, const uint16_t steps, const int16_t speed) {}

void MksTest::onSeekPosition(const uint16_t motor, const int32_t position, const int16_t speed) {}

void MksTest::onGetPosition(const uint16_t motor, const int32_t position) {
    std::cout << std::dec << "(Queried)   Motor " << motor << ": position=" << position << std::endl;
}