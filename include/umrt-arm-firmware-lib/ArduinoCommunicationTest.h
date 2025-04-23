//
// Created by Noah on 2024-06-13.
//

#ifndef UMRT_ARM_FIRMWARE_LIB_COMMUNICATION_TEST_H
#define UMRT_ARM_FIRMWARE_LIB_COMMUNICATION_TEST_H

#include "StepperController.h"
#include "Utils.h"
#include <string>
#include <thread>

#include <vector>

// TODO: Could use some docs, would be nice if we could get some sample output
class ArduinoCommunicationTest {
public:
    ArduinoCommunicationTest(const std::string& device, const int baud, const std::vector<uint8_t>& motor_ids) : motor_ids(motor_ids) {
        s.ESetup.connect([this] { this->onSetup(); });
        s.EStringReceived.connect([this](std::string&& str) { this->onString(std::forward<decltype(str)>(str)); });
        s.EArduinoEcho.connect([this](std::vector<uint8_t>&& p) { this->onEcho(std::forward<decltype(p)>(p)); });
        s.ESetSpeed.connect([this](auto&& PH1, auto&& PH2) {
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
        });
        s.EGetPosition.connect([this](auto&& PH1, auto&& PH2) {
            onGetPosition(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
        });
        s.connect(device, baud);
    }

    void update() { return s.update(); }

    void sendTestRoutine() {
        s.sendString("test");

        // Wait 1 second
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Setup handler for text echos and send one
        processPayload = [this](const std::vector<uint8_t>& p) -> void { this->onEchoText(p); };
        s.sendEcho(encode_string("hello world"));

        // Wait 1 second
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Setup handler for 32-bit numerical echos and send 3
        processPayload = [this](const std::vector<uint8_t>& p) -> void { this->onEchoInt32(p); };
        s.sendEcho(pack_32(0xDEAD'BEEF));
        s.sendEcho(pack_32(1000));
        s.sendEcho(pack_32(32767));

        // Wait 1 second
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Setup handler for raw 32-bit numerical echos and send 3
        processPayload = [this](const std::vector<uint8_t>& p) -> void { this->onEchoRaw(p); };
        s.sendEcho(pack_32(0xDEAD'BEEF));
        s.sendEcho(pack_32(1000));
        s.sendEcho(pack_32(32767));

        // Wait 1 second
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Test motors
        for (uint8_t motor : this->motor_ids) {
            // Send speed of 2 RPM for 5 seconds, then 1 RPM in other direction for 5 seconds, then stop
            s.getPosition(motor);
            s.setSpeed(motor, 20);
            s.getSpeed(motor);
            std::this_thread::sleep_for(std::chrono::seconds(5));
            s.setSpeed(motor, -10);
            s.getSpeed(motor);
            std::this_thread::sleep_for(std::chrono::seconds(5));
            s.setSpeed(motor, 0);
            s.getSpeed(motor);

            //Step forward 20 steps at 10 RPM, then back 10 steps at 5 RPM
            s.getPosition(motor);
            s.sendStep(motor, 20, 100);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            s.getPosition(motor);
            s.sendStep(motor, 10, -50);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            s.getPosition(motor);

            // Wait 1 second
            std::this_thread::sleep_for(std::chrono::seconds(1));

            // Seek back to position 0 from wherever we ended up at 10 RPM
            s.seekPosition(motor, 0, 100);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            s.getPosition(motor);

            // Wait 1 second
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

protected:
    StepperController s;
    std::thread test_thread;
    const std::vector<uint8_t> motor_ids;

    void onSetup() {
        std::cout << "Arduino setup!" << std::endl;

        // Start the test procedure
        test_thread = std::thread(&ArduinoCommunicationTest::sendTestRoutine, this);
    }

    void onString(const std::string& str) {
        std::cout << str << std::endl;
    }

    void onEcho(const std::vector<uint8_t>& payload) {
        this->processPayload(payload);
    }

    void onEchoText(const std::vector<uint8_t>& payload) {
        std::cout << decode_string(payload) << std::endl;
    }

    void onEchoInt32(const std::vector<uint8_t>& payload) {
        std::cout << decode_32(payload) << std::endl;
    }

    void onEchoRaw(const std::vector<uint8_t>& payload) {
        if (payload.empty()) { return; }

        std::cout << "[ 0x" << std::hex << std::setw(2) << std::setfill('0') << +payload[0];
        for (auto p = payload.cbegin() + 1; p != payload.end(); ++p) {
            std::cout << ", 0x" << std::hex << std::setw(2) << std::setfill('0') << +*p;
        }
        std::cout << " ]" << std::endl;
    }

    void onSetSpeed(const uint8_t motor, const int16_t speed) {
        // motor casted to uint16_t so that it outputs as an integer instead of a char
        std::cout << std::dec << "(Requested) Motor " << (uint16_t)motor << ": speed=" << speed << std::endl;
    }

    void onGetSpeed(const uint8_t motor, const int16_t speed) {
        // motor casted to uint16_t so that it outputs as an integer instead of a char
        std::cout << std::dec << "(Queried)   Motor " << (uint16_t)motor << ": speed=" << speed << std::endl;
    }

    void onSendStep(const uint8_t motor, const uint16_t steps, const int16_t speed) {
        // motor casted to uint16_t so that it outputs as an integer instead of a char
        std::cout << std::dec << "(Requested) Motor " << (uint16_t)motor << ": steps=" << steps << ", speed=" << speed << std::endl;
    }

    void onSeekPosition(const uint8_t motor, const int32_t position, const int16_t speed) {
        // motor casted to uint16_t so that it outputs as an integer instead of a char
        std::cout << std::dec << "(Requested) Motor " << (uint16_t)motor << ": position=" << position << ", speed=" << speed << std::endl;
    }

    void onGetPosition(const uint8_t motor, const int32_t position) {
        // motor casted to uint16_t so that it outputs as an integer instead of a char
        std::cout << std::dec << "(Queried)   Motor " << (uint16_t)motor << ": position=" << position << std::endl;
    }

private:
    std::function<void(const std::vector<uint8_t>&)> processPayload;
};


#endif //UMRT_ARM_FIRMWARE_LIB_COMMUNICATION_TEST_H
