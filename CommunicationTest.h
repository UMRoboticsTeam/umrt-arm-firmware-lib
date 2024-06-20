//
// Created by Noah on 2024-06-13.
//

#ifndef COMMUNICATION_MASTER_EXAMPLE_COMMUNICATIONTEST_H
#define COMMUNICATION_MASTER_EXAMPLE_COMMUNICATIONTEST_H

#include "StepperController.h"
#include "Utils.h"
#include <boost/log/trivial.hpp>
#include <thread>
#include <string>

#include <vector>

class CommunicationTest {
public:
    CommunicationTest(const std::string& device, const int baud) {
        s.ESetup.connect([this] { this->onSetup(); });
        s.EStringReceived.connect([this](std::string&& str) { this->onString(std::forward<decltype(str)>(str)); });
        s.EArduinoEcho.connect([this](std::vector<uint8_t>&& p) { this->onEcho(std::forward<decltype(p)>(p)); });
        s.connect(device, baud);
    }

    void update() { return s.update(); }

    void sendTestRoutine(){
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

        // Send speed of 2 RPM for 5 seconds, then 1 RPM in other direction for 5 seconds, then stop
        s.setSpeed(20);
        std::this_thread::sleep_for(std::chrono::seconds(5));
        s.setSpeed(-10);
        std::this_thread::sleep_for(std::chrono::seconds(5));
        s.setSpeed(0);
    }

protected:
    StepperController s;
    std::thread test_thread;

    void onSetup() {
        std::cout << "Arduino setup!" << std::endl;

        // Start the test procedure
        test_thread = std::thread(&CommunicationTest::sendTestRoutine, this);
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

private:
    std::function<void(const std::vector<uint8_t>&)> processPayload;
};


#endif //COMMUNICATION_MASTER_EXAMPLE_COMMUNICATIONTEST_H
