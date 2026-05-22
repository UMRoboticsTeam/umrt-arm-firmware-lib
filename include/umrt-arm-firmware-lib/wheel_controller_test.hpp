#ifndef UMRT_WHEEL_CONTROLLER_HPP
#define UMRT_WHEEL_CONTROLLER_HPP

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

#include "wheel_controller.hpp"
#include <thread>
#include <atomic>
#include <iostream>
#include "ROVER_COMMANDS.hpp"


class Wheel_Controller_test {
public:

    wheel_controller_test(const std::string& can_interface);
    void runTestRoutine();
    void update();

protected:
    WheelController controller;
    std::thread test_thread;
    std::atomic<bool> stop_flag;
    void printPayload(const std::array<uint8_t, 8>& payload);
};

#endif 
