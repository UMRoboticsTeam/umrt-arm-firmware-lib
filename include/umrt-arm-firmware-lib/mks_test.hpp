//
// Created by Noah on 2024-06-13.
//

#ifndef UMRT_ARM_FIRMWARE_LIB_MKS_TEST_HPP
#define UMRT_ARM_FIRMWARE_LIB_MKS_TEST_HPP

#include "MKS_COMMANDS.hpp"
#include "mks_stepper_controller.hpp"

#include <string>
#include <thread>
#include <vector>

// TODO: Could use some docs, would be nice if we could get some sample output
class MksTest {
public:
    MksTest(const std::string& can_interface, std::shared_ptr<const std::unordered_set<uint16_t>> motor_ids,
            const uint8_t norm_factor = 1);

    void update();

    void sendTestRoutine();

protected:
    MksStepperController s;
    std::thread test_thread;
    std::shared_ptr<const std::unordered_set<uint16_t>> motor_ids;

    void onSetSpeed(const uint16_t motor, const bool status);

    void onSendStep(const uint16_t motor, const MksMoveResponse status);

    void onSeekPosition(const uint16_t motor, const MksMoveResponse status);

    void onGetPosition(const uint16_t motor, const int32_t position);
};


#endif //UMRT_ARM_FIRMWARE_LIB_MKS_TEST_HPP
