//
// Created by Noah on 2024-06-13.
//

#ifndef UMRT_ARM_FIRMWARE_LIB_COMMUNICATION_TEST_H
#define UMRT_ARM_FIRMWARE_LIB_COMMUNICATION_TEST_H

#include "ArduinoStepperController.h"
#include "Utils.h"
#include <string>
#include <thread>

#include <vector>

// TODO: Could use some docs, would be nice if we could get some sample output
class ArduinoCommunicationTest {
public:
    ArduinoCommunicationTest(const std::string& device, const int baud, const std::vector<uint8_t>& motor_ids);

    void update();

    void sendTestRoutine();

protected:
    ArduinoStepperController s;
    std::thread test_thread;
    const std::vector<uint8_t> motor_ids;

    void onSetup();

    void onString(const std::string& str);

    void onEcho(const std::vector<uint8_t>& payload);

    void onEchoText(const std::vector<uint8_t>& payload);

    void onEchoInt32(const std::vector<uint8_t>& payload);

    void onEchoRaw(const std::vector<uint8_t>& payload);

    void onSetSpeed(const uint8_t motor, const int16_t speed);

    void onGetSpeed(const uint8_t motor, const int16_t speed);

    void onSendStep(const uint8_t motor, const uint16_t steps, const int16_t speed);

    void onSeekPosition(const uint8_t motor, const int32_t position, const int16_t speed);

    void onGetPosition(const uint8_t motor, const int32_t position);

private:
    std::function<void(const std::vector<uint8_t>&)> processPayload;
};


#endif //UMRT_ARM_FIRMWARE_LIB_COMMUNICATION_TEST_H
