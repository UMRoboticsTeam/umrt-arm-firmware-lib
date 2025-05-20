//
// Created by Noah on 2024-06-13.
//

#ifndef UMRT_ARM_FIRMWARE_LIB_MKS_TEST_HPP
#define UMRT_ARM_FIRMWARE_LIB_MKS_TEST_HPP

#include "mks_stepper_controller.hpp"
#include "utils.hpp"
#include <string>
#include <thread>

#include <vector>

// TODO: Could use some docs, would be nice if we could get some sample output
class MksTest {
public:
    MksTest(const std::string& can_interface, const std::vector<uint8_t>& motor_ids, const uint8_t norm_factor = 1);

    void update();

    void sendTestRoutine();

protected:
    MksStepperController s;
    std::thread test_thread;
    const std::vector<uint8_t> motor_ids;

    void onSetSpeed(const uint16_t motor, const int16_t speed);

    void onGetSpeed(const uint16_t motor, const int16_t speed);

    void onSendStep(const uint16_t motor, const uint16_t steps, const int16_t speed);

    void onSeekPosition(const uint16_t motor, const int32_t position, const int16_t speed);

    void onGetPosition(const uint16_t motor, const int32_t position);
};


#endif //UMRT_ARM_FIRMWARE_LIB_MKS_TEST_HPP
