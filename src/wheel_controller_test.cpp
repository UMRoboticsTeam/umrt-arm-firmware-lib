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

#include "wheel_controller.hpp"

class WheelControllerTest{

    WheelControllerTest(const std::string& can_interface)
        : controller(can_interface), stop_flag(false)
    {
        std::cout << "WheelControllerTest setup!" << std::endl;

        // Start the background test thread
        test_thread = std::thread(&runTestRoutine, this);
    }

    //safe thread shutdown
    /**
    ~WheelControllerTest() {
        stop_flag = true;
        if (test_thread.joinable()) { //check thread is actually running, not yet joined, not detached
            test_thread.join(); 
        }
    }
    */

    //peridoically call update() so incoming CAN messages are processed
    void  update() {
        try {
            controller.update(std::chrono::milliseconds(10));
        } catch (...) {
            // Ignore timeouts
        }
    }


    WheelController controller;
    std::thread test_thread;
    std::atomic<bool> stop_flag;

    //main test routine, running in background thread
    void runTestRoutine() {
        // Give the CAN interface a moment to initialize
        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::cout << "=== Starting Rover Speed Test Sequence ===" << std::endl;

        while (!stop_flag) {

            //slow forward command
            int16_t left = 10;   // 10 rpm forward
            int16_t right = 10;  // 10 rpm forward
            uint32_t priority = 7;  // priority 7 (set/adjust)

            std::cout << "(Requested) Rover: SetSpeed L=" << left
                        << " rpm, R=" << right
                        << " rpm, priority=" << priority << std::endl;

            // Send the command
            controller.setSpeed(left, right, priority);

            std::this_thread::sleep_for(std::chrono::seconds(2));
                    


            //differential turn command (left wheel forward, right wheel reverse)
            left = 20;    // forward left wheel
            right = -20;  // reverse right wheel
            priority = 7;

            std::cout << "(Requested) Rover: SetSpeed L=" << left
                        << " rpm, R=" << right
                        << " rpm, priority=" << priority << std::endl;

            controller.setSpeed(left, right, priority);


            std::this_thread::sleep_for(std::chrono::seconds(2));

            //stop command
            left = 0;
            right = 0;
            priority = 7;

            std::cout << "(Requested) Rover: SetSpeed L=" << left
                << " rpm, R=" << right
                << " rpm, priority=" << priority << std::endl;

            controller.setSpeed(left, right, priority);

            std::this_thread::sleep_for(std::chrono::seconds(2));

            //process any incoming CAN messages
            update();
        }
    }
};

int main() {
    WheelControllerTest test("/dev/can0");

    // Main thread can do other work here, or just wait
    std::this_thread::sleep_for(std::chrono::seconds(30)); // Run the test for 30 seconds

    return 0;
}
