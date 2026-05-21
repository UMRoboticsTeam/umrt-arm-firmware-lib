/**
 * - starts a background thread
 * - sends rover speeds periodically
 * - print every action to consol for debug
 * - calls update() to process incoming CAN messages
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <atomic>
#include "wheel_controller.hpp"   //updated wheel_controller.hpp 2026


class WheelControllerTest {
public:
    WheelControllerTest(const std::string& can_interface)
        : controller(can_interface), stop_flag(false)
    {
        std::cout << "WheelControllerTest setup!" << std::endl;

        // Start the background test thread
        test_thread = std::thread(&WheelControllerTest::runTestRoutine, this);
    }

    //safe thread shutdown
    ~WheelControllerTest() {
        stop_flag = true;
        if (test_thread.joinable()) { //check thread is actually running, not yet joined, not detached
            test_thread.join(); 
        }
    }

    //peridoically call update() so incoming CAN messages are processed
    void update() {
        try {
            controller.update(std::chrono::milliseconds(10));
        } catch (...) {
            // Ignore timeouts
        }
    }

private:
    WheelController controller;
    std::thread test_thread;
    std::atomic<bool> stop_flag;

    //prints outgoing payload in readable format for debug
    void printPayload(const std::array<uint8_t, 8>& payload) {
        std::cout << "  Payload: [ ";
        for (auto b : payload) {
            std::cout << "0x" << std::hex << (int)b << " ";
        }
        std::cout << std::dec << "]" << std::endl;
    }

    //main test routine, running in background thread
    void runTestRoutine() {
        // Give the CAN interface a moment to initialize
        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::cout << "=== Starting Rover Speed Test Sequence ===" << std::endl;

        while (!stop_flag) {

            //slow forward command
            {
                int16_t left = 10;   // 10 rpm forward
                int16_t right = 10;  // 10 rpm forward
                uint32_t priority = 7;  // priority 7 (set/adjust)

                std::cout << "(Requested) Rover: SetSpeed L=" << left
                          << " rpm, R=" << right
                          << " rpm, priority=" << priority << std::endl;

                // Send the command
                controller.setSpeed(left, right, priority);

                //Print payload
                std::array<uint8_t, 8> payload;
                packPayload(payload, left, right, 0); // counter ignored for print
                printPayload(payload);

                std::this_thread::sleep_for(std::chrono::seconds(2));
            }


            //differential turn command (left wheel forward, right wheel reverse)
            {
                int16_t left = 20;    // forward left wheel
                int16_t right = -20;  // reverse right wheel
                uint32_t priority = 7;

                std::cout << "(Requested) Rover: SetSpeed L=" << left
                          << " rpm, R=" << right
                          << " rpm, priority=" << priority << std::endl;

                controller.setSpeed(left, right, priority);

                std::array<uint8_t, 8> payload;
                packPayload(payload, left, right, 0);
                printPayload(payload);

                std::this_thread::sleep_for(std::chrono::seconds(2));
            }

            //stop command
            {
                int16_t left = 0;
                int16_t right = 0;
                uint32_t priority = 7;

                std::cout << "(Requested) Rover: SetSpeed L=" << left
                          << " rpm, R=" << right
                          << " rpm, priority=" << priority << std::endl;

                controller.setSpeed(left, right, priority);

                std::array<uint8_t, 8> payload;
                packPayload(payload, left, right, 0);
                printPayload(payload);

                std::this_thread::sleep_for(std::chrono::seconds(2));
            }

            //process any incoming CAN messages
            update();
        }
    }
};


