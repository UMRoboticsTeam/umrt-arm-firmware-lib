/**
 * THis class implements the rover-speed CAN interface using J1939 style 29-bit CAN identifier
 * Sends left/right wheel speeds to STM32 drivetrain controller and processes any responses
 * 
 * - pure rover-speed CAN interface
 * - no motor ID filtering, no support to mks stepper drivers
 */

#ifndef UMRT_WHEEL_CONTROLLER_HPP
#define UMRT_WHEEL_CONTROLLER_HPP

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

namespace drivers::socketcan {
    class SocketCanReceiver;
    class SocketCanSender;
    class CanId;
}


class WheelController {
public:

    /**
     * Constructor
     *
     * @param can_interface  The SocketCAN interface name (e.g., "can0", "vcan0")
     */
    explicit WheelController(const std::string& can_interface);

    //Destructor
    ~WheelController() noexcept;

    /**
     * Sends a rover-speed command to the STM32.
     *
     * @param left_speed   Signed left wheel speed in RPM
     * @param right_speed  Signed right wheel speed in RPM
     * @param priority     J1939 priority (0–7). Priority 0 = emergency stop
     *
     * @return true if the CAN frame was successfully transmitted
     */
    bool setSpeed(int16_t left_speed, int16_t right_speed, uint32_t priority);

    //Returns true if the CAN interface was successfully initialized
    bool isSetup() const;

    /**
     * Polls the CAN bus for incoming messages
     *
     * @param timeout  Max wait time for a CAN frame
     */
    void update(const std::chrono::nanoseconds& timeout);

private:

    //Handles any incoming CAN message addressed to this controller
    void handleCANMessage(const std::vector<uint8_t>& message,
                          drivers::socketcan::CanId& info);

    //Handles ECHO responses from the STM32
    void handleEcho(const std::vector<uint8_t>& message,
                    drivers::socketcan::CanId& info);

    //Handles GET_SPEED responses from the STM32
    void handleGetSpeed(const std::vector<uint8_t>& message,
                        drivers::socketcan::CanId& info);

private:
    // CAN I/O
    std::unique_ptr<drivers::socketcan::SocketCanReceiver> can_receiver;
    std::unique_ptr<drivers::socketcan::SocketCanSender>   can_sender;

    // True once CAN interface is initialized
    bool setup_completed = false;

    // Message counter for rover-speed command (0–250, rollover at 251)
    uint8_t msg_counter = 0;
};

#endif 
