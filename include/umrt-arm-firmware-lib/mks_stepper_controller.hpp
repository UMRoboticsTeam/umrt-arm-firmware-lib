//
// Created by Noah Reeder on 2025-04-22
//

#ifndef UMRT_ARM_FIRMWARE_LIB_MKS_STEPPER_CONTROLLER_HPP
#define UMRT_ARM_FIRMWARE_LIB_MKS_STEPPER_CONTROLLER_HPP

#include <boost/signals2.hpp>
#include <string>
#include <vector>

// Forward declaring these classes so that ros2_socketcan can be a private dependency
namespace drivers::socketcan {
    class SocketCanReceiver;
    class SocketCanSender;
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
class MksStepperController {
public:
    /**
     * Initializes an MksStepperController.
     *
     * @param can_interface the SocketCAN network interface corresponding to the CAN bus
     * @param norm_factor the interpolation normalisation factor to use, see @ref internorm; defaults to off
     */
    MksStepperController(const std::string& can_interface, const uint8_t norm_factor = 1);

    /**
     * Destroys an MksStepperController.
     */
    ~MksStepperController() noexcept;

    /**
     * Sends a @ref MksTest.Commands.SET_SPEED command to set the speed of a motor.
     * Response callbacks are available through @ref ESetSpeed.
     * See @ref MksTest.Constants.MAX_SPEED for speed limits.
     *
     * @param motor the ID of the motor to control
     * @param speed the signed target speed to set the motor to, in RPM
     * @param acceleration the speed ramp profile, see @ref MksTest.Constants.MAX_ACCEL; defaults to instantaneous
     * @return `true` if transmitted over the CAN bus
     */
    bool setSpeed(const uint16_t motor, const int16_t speed, const uint8_t acceleration = 0);

    /**
     * Sends a @ref MksTest.Commands.MOTOR_SPEED command to query the speed of a motor.
     * Response callbacks are available through @ref EGetSpeed.
     *
     * @param motor the ID of the motor to query
     * @return `true` if transmitted over the CAN bus
     */
    bool getSpeed(const uint16_t motor);

    /**
     * Sends a @ref MksTest.Commands.SEND_STEP command to move a motor a fixed number of steps.
     * Direction is controlled by the sign of the target speed.
     * Response callbacks are available through @ref ESendStep.
     *
     * @param motor the ID of the motor to move
     * @param num_steps the number of steps to move, maximum of 2^24 - 1
     * @param speed the signed target speed to set the motor to, in RPM
     * @param acceleration the speed ramp profile, see @ref MksTest.Constants.MAX_ACCEL; defaults to instantaneous
     * @return `true` if transmitted over the CAN bus
     */
    bool sendStep(const uint16_t motor, const uint32_t num_steps, const int16_t speed, const uint8_t acceleration = 0);

    /**
     * Sends a @ref MksTest.Commands.SEEK_POS_BY_STEPS command to move a motor to specific step position.
     * Since this command seeks a position, the sign of the speed is ignored.
     *
     * @param motor the ID of the motor to move
     * @param position the target position in number of steps from the motor's zero point, maximum of 2^23 - 1
     * @param speed the signed target speed to set the motor to, in RPM; note that the absolute value is taken
     * @param acceleration the speed ramp profile, see @ref MksTest.Constants.MAX_ACCEL; defaults to instantaneous
     * @return `true` if transmitted over the CAN bus
     */
    bool seekPosition(const uint16_t motor, const int32_t position, const int16_t speed, const uint8_t acceleration = 0);

    /**
      * Sends a @ref MksTest.Commands.CURRENT_POS command to query the current position of a motor in steps.
      *
      * @param motor the ID of the motor to query
      * @return `true` if transmitted over the CAN bus
      */
    bool getPosition(const uint16_t motor);

    /**
     * Returns whether the CAN bus connection has been fully established.
     * @return `true` if so
     */
    [[nodiscard]] bool isSetup() const;

    /**
     * Polls for CAN messages.
     * If an applicable message is received, the appropriate event is signalled.
     *
     * @param timeout maximum time to wait for a message to appear on the bus
     */
    void update(const std::chrono::nanoseconds & timeout = std::chrono::nanoseconds::zero());

    // ==========================
    //           Events
    // ==========================

    /**
     * <a href=https://www.boost.org/doc/libs/1_63_0/doc/html/signals.html>Boost signal</a> triggered when
     * @ref setSpeed responses are received.
     */
    boost::signals2::signal<void(uint16_t, int16_t)> ESetSpeed;

    /**
     * <a href=https://www.boost.org/doc/libs/1_63_0/doc/html/signals.html>Boost signal</a> triggered when
     * @ref getSpeed responses are received.
     */
    boost::signals2::signal<void(uint16_t, int16_t)> EGetSpeed;

    /**
     * <a href=https://www.boost.org/doc/libs/1_63_0/doc/html/signals.html>Boost signal</a> triggered when
     * @ref sendStep responses are received.
     */
    boost::signals2::signal<void(uint16_t, uint16_t, int16_t)> ESendStep;

    /**
     * <a href=https://www.boost.org/doc/libs/1_63_0/doc/html/signals.html>Boost signal</a> triggered when
     * @ref seekPosition responses are received.
     */
    boost::signals2::signal<void(uint16_t, int32_t, int16_t)> ESeekPosition;

    /**
     * <a href=https://www.boost.org/doc/libs/1_63_0/doc/html/signals.html>Boost signal</a> triggered when
     * @ref getPosition responses are received.
     *
     * @param 1st [uint8_t] motor ID
     * @param 2nd [int32_t] motor position in steps
     */
    boost::signals2::signal<void(uint16_t, int32_t)> EGetPosition;

protected:
    /**
     * Handles received CAN messages and sends out signals as appropriate.
     *
     * @param message the message payload
     * @param info auxiliary information associated with the message, e.g. driver ID, bus time
     */
    void handleCanMessage(const std::vector<uint8_t>& message, drivers::socketcan::CanId & info);

    /**
     * @name Signal Processing Helper Functions
     * Helper functions for decoding the parameters of Sysex commands processed by @ref handleSysex before forwarding
     * to their associated <a href=https://www.boost.org/doc/libs/1_63_0/doc/html/signals.html>signal</a>.
     *
     * @param message the de-firmatified Sysex payload
     */
    //@{
    void handleESetSpeed(const std::vector<unsigned char>& message);

    void handleEGetSpeed(const std::vector<unsigned char>& message);

    void handleESendStep(const std::vector<unsigned char>& message);

    void handleESeekPosition(const std::vector<unsigned char>& message);

    void handleEGetPosition(const std::vector<unsigned char>& message, drivers::socketcan::CanId & info);
    //@}

    std::unique_ptr<drivers::socketcan::SocketCanReceiver> can_receiver;
    std::unique_ptr<drivers::socketcan::SocketCanSender> can_sender;
    const uint8_t norm_factor;

private:
    /**
     * Flag which indicates whether the CAN bus connection has been initialised.
     */
    bool setup_completed;
};

#endif //UMRT_ARM_FIRMWARE_LIB_MKS_STEPPER_CONTROLLER_HPP
