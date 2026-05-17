//
// Created by Niko Christie on 2025-07-01
//

#ifndef UMRT_ARM_FIRMWARE_LIB_WHEEL_CONTROLLER_HPP
#define UMRT_ARM_FIRMWARE_LIB_WHEEL_CONTROLLER_HPP

// #include <boost/signals2.hpp>
#include <chrono>
#include <string>
#include <unordered_set>
#include <vector>

// #include "mks_enums.hpp"

// Forward declaring these classes so that ros2_socketcan can be a private dependency
namespace drivers::socketcan {
    class SocketCanReceiver;
    class SocketCanSender;
    class CanId;
} // namespace drivers::socketcan

/**
 * Abstracts CAN bus communication to MKS SERVO57D/42D/35D/28D stepper motor driver modules. Responses are conveyed through
 * <a href=https://www.boost.org/doc/libs/1_63_0/doc/html/signals.html>Boost signals</a>.
 *
 * # Interpolated Normalisation {#internorm}
 * To get around the limitations introduced by specifying speeds under [nominal](@ref noc) conditions, a technique
 * which will be referred to as "interpolated normalisation" is used to normalise the units of speed to RPM, assuming
 * 200 full-steps per revolution. In other words, the unit of speed shifts from 160/3 steps/s to 200 steps/min.
 *
 * This is achieved by micro-stepping the motor at a specified interpolation factor. For example, if an interpolation
 * factor of 16 is used, and the motor is requested to move at 2 RPM, the motor will actually move at 6400 steps/min.
 *
 * An interpolation factor of 1 can be used to disable interpolation.
 */
class WheelController {
public:
    /**
     * Initializes an WheelController.
     */
    WheelController(const std::string& can_interface);

    /**
     * Destroys an WheelController.
     */
    ~WheelController() noexcept;

    /**
     * Sends a @ref RoverCommands::SET_SPEED command to set the speed ofthe motors.
     * Response callbacks are available through @ref ESetSpeed.
     * See @ref MksTest.Constants.MAX_SPEED for speed limits.
     *
     * @param left_speed speed value of the left wheels 
     * @param right_speed speed value of the right wheels
     * @param priority priority for CAN message    
     * @return `true` if transmitted over the CAN bus
     */
    bool setSpeed(const int16_t left_speed, const int16_t right_speed, const uint32_t priority);

    /**
      * Sends a @ref MksCommands::CURRENT_POS command to query the current position of a motor in steps.
      *
      * @param motor the ID of the motor to query
      * @return `true` if transmitted over the CAN bus
      */
    // bool getSpeed(const uint16_t motor);

    /**
     * Returns whether the CAN bus connection has been fully established.
     * @return `true` if so
     */
    [[nodiscard]] bool isSetup() const;

    /**
     * Polls for CAN messages.
     * If an applicable message is received, the appropriate CAN message handler is called.
     *
     * @param timeout maximum time to wait for a message to appear on the bus
     */
    void update(const std::chrono::nanoseconds& timeout = std::chrono::nanoseconds::zero());

protected:

    /**
     * Handles received CAN messages and calls command handler.
     *
     * @param message the message payload
     * @param info auxiliary information associated with the message, e.g. driver ID, bus time
     */
    void handleCANMessage(const std::vector<uint8_t>& message, drivers::socketcan::CanId& info);

    /**
     * @name Signal Processing Helper Functions
     * Helper functions for decoding the parameters of Sysex commands processed by @ref handleSysex before forwarding
     * to their associated <a href=https://www.boost.org/doc/libs/1_63_0/doc/html/signals.html>signal</a>.
     *
     * @param message the de-firmatified Sysex payload
     */
    //@{
    void handleEcho(const std::vector<uint8_t>& message, drivers::socketcan::CanId& info);

    void handleGetSpeed(const std::vector<uint8_t>& message, drivers::socketcan::CanId& info);

    // void handleEGetPosition(const std::vector<unsigned char>& message, drivers::socketcan::CanId& info);
    //@}

    std::unique_ptr<drivers::socketcan::SocketCanReceiver> can_receiver;
    std::unique_ptr<drivers::socketcan::SocketCanSender> can_sender;

private:
    /**
     * Flag which indicates whether the CAN bus connection has been initialised.
     */
    bool setup_completed;

    /**
     * Message Counter for J1939 Payload, allow us to detect if messages are being lost
     */
    uint8_t msg_counter;
};

#endif //UMRT_ARM_FIRMWARE_LIB_WHEEL_CONTROLLER_HPP
