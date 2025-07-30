//
// Created by Noah on 2025-07-30.
//

#ifndef UMRT_ARM_FIRMWARE_LIB_SERVO_CONTROLLER_HPP
#define UMRT_ARM_FIRMWARE_LIB_SERVO_CONTROLLER_HPP

#include <string>
#include <memory>

// Forward declaring these classes so that ros2_socketcan can be a private dependency
namespace drivers::socketcan {
    class SocketCanReceiver;
    class SocketCanSender;
    class CanId;
} // namespace drivers::socketcan

/**
 * Abstracts CAN bus communication to a CAN-PWM gateway.
 * It is intended that this will one day support multiple servos, likely using DroneCAN's
 * uavcan.equipment.actuator.ArrayCommand messages to communicate with a Mateksys CAN-L4-PWM gateway.
 */
class ServoController {
public:
    /**
     * Initializes an ServoController.
     *
     * @param can_interface SocketCAN network interface corresponding to the CAN bus
     * @param servo_id CAN ID corresponding to the gateway's command interface
     */
    ServoController(
            const std::string& can_interface,
            const uint16_t servo_id
    );

    /**
     * Destroys an ServoController.
     */
    ~ServoController() noexcept;

    /**
     * Command a servo position.
     * @param position position with [0, 255] mapping to the gateway's full servo range
     * @return `true` if transmitted over the CAN bus
     */
    bool send(const uint8_t position);

    /**
     * Returns whether the CAN bus connection has been fully established.
     * @return `true` if so
     */
    [[nodiscard]] bool isSetup() const;

protected:
    const uint16_t servo_id_;
    std::unique_ptr<drivers::socketcan::SocketCanSender> can_sender_;

private:
    /**
     * Flag which indicates whether the CAN bus connection has been initialised.
     */
    bool setup_completed_;
};

#endif //UMRT_ARM_FIRMWARE_LIB_SERVO_CONTROLLER_HPP
