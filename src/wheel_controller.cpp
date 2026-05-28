//
// Created by Niko Christie on 2025-07-01.
//

#include <boost/log/trivial.hpp>

#include <ros2_socketcan/socket_can_receiver.hpp>
#include <ros2_socketcan/socket_can_sender.hpp>

#include <numeric>

#include "ROVER_COMMANDS.hpp"
#include "wheel_controller.hpp"
#include "Crc8_J1850.h"
#include "utils.hpp"
#include <cmath>
#include <array>
#include <algorithm>

/**
 * Packs the speed onto payload
 * @param payload std::vector<uint8_t> to append the properties structure to
 * @param left_speed speed value of the left wheels 
 * @param right_speed speed value of the right wheels 
 * @param counter message counter for STM32 to check for missed messages 
 */
namespace {
    void packPayload(std::array<uint8_t, 8>& payload, const int16_t left_speed, const int16_t right_speed, uint8_t counter) {

        //  CONSTANTS for SLOT 
        constexpr double OFFSET = -4016.0;
        constexpr double MIN_RPM = -4016.0;
        constexpr double MAX_RPM = 4015.875;
        constexpr double SCALE = 0.125;

        //  Using doubles for this because of the scaling
        //  Apply SLOT to speeds - Scaling, Limits, Offset and Transfer Function
        const double left_speed_d = std::clamp(static_cast<double>(left_speed), MIN_RPM, MAX_RPM);
        const double right_speed_d = std::clamp(static_cast<double>(right_speed), MIN_RPM, MAX_RPM);    

        const uint16_t raw_left  = static_cast<uint16_t>(std::round((left_speed_d - OFFSET) / SCALE));
        const uint16_t raw_right = static_cast<uint16_t>(std::round((right_speed_d - OFFSET) / SCALE));

        //  Pack Speed Data (Bytes 0-3)
        payload[0] = static_cast<uint8_t>(raw_left & 0xFF);
        payload[1] = static_cast<uint8_t>((raw_left >> 8) & 0xFF);
        payload[2] = static_cast<uint8_t>(raw_right & 0xFF);
        payload[3] = static_cast<uint8_t>((raw_right >> 8) & 0xFF);

        //  Bytes 4-5 stay 0x00 - for now 
        payload[4] = 0xFF;
        payload[5] = 0xFF;

        //  Message Counter (Byte 6)
        payload[6] = counter;

        //  Checksum (Byte 7)
        //  This is because Crc8 uses uint8*
        payload[7] = Crc8(payload.data(), 8, 0xFF);    
    }   //  packPayload()
}

WheelController::WheelController(const std::string& can_interface) {

    BOOST_LOG_TRIVIAL(trace) << "WheelController construction begun.";

    this->can_receiver = std::make_unique<drivers::socketcan::SocketCanReceiver>(can_interface);
    this->can_sender = std::make_unique<drivers::socketcan::SocketCanSender>(can_interface);

    msg_counter = 0;

    BOOST_LOG_TRIVIAL(debug) << "WheelController constructed.";

    setup_completed = true;
}

WheelController::~WheelController() noexcept { BOOST_LOG_TRIVIAL(debug) << "WheelController destructed."; }

// Included for posterity; due to strange responses this command is assumed to return the encoder speed, and we are not
// using the driver's encoder so I cannot test this and it is not useful functionality anyways
//bool WheelController::getSpeed(const uint16_t motor) {
//    if (!isSetup()) { return false; }
//
//    std::vector<uint8_t> payload{ MksCommands::MOTOR_SPEED };
//    payload.insert(payload.end(), checksum(motor, payload));
//
//    try {
//        drivers::socketcan::CanId can_id(motor, 0, drivers::socketcan::FrameType::DATA, drivers::socketcan::StandardFrame);
//        can_sender->send(payload.data(), payload.size(), can_id);
//    } catch (drivers::socketcan::SocketCanTimeout& e) {
//        BOOST_LOG_TRIVIAL(warning) << "WheelController getSpeed timeout: motor=" << motor;
//        return false;
//    }
//
//    return true;
//}

// bool WheelController::getSpeed(const uint16_t motor) {
//     if (!isSetup()) { return false; }

//     std::vector<uint8_t> payload{ MksCommands::CURRENT_POS };
//     payload.insert(payload.end(), checksum(motor, payload));

//     try {
//         drivers::socketcan::CanId can_id(motor, 0, drivers::socketcan::FrameType::DATA, drivers::socketcan::StandardFrame);
//         can_sender->send(payload.data(), payload.size(), can_id);
//     } catch (drivers::socketcan::SocketCanTimeout& e) {
//         BOOST_LOG_TRIVIAL(warning) << "WheelController getPosition timeout: motor=0x" << std::hex << motor << std::dec;
//         return false;
//     }
//     return true;
// }

bool WheelController::setSpeed(const int16_t left_speed, const int16_t right_speed, const uint32_t priority){

    if (!isSetup()) { return false; }

    // //  Can't let the speeds be 0xFFFF or 0xFFFE, let's log to see which one it is
    // if (left_speed == 0xFFFF || right_speed == 0xFFFF) {
    //     BOOST_LOG_TRIVIAL(error) << "ERROR -> Left Speed: " << std::hex 
    //                              << left_speed <<  ", Right Speed: " << right_speed;
    //     return false; 
    // }
    // if (left_speed == 0xFFFE || right_speed == 0xFFFE) {
    //     BOOST_LOG_TRIVIAL(debug) << "DONT CARE -> Left Speed: " << std::hex 
    //                              << left_speed <<  ", Right Speed: " << right_speed;
    //     return false; 
    // }

    //  Setup CAN ID
    uint32_t rdp = 0;
    uint32_t pf = 0xFF; 
    uint32_t ps = RoverCommands::SET_SPEED; //  Command for Set Wheel Speed
    uint32_t source = 0x80;  //  Source Address for Jetson Xavier AGX placeholder - ea

    uint32_t ID = (priority << 26) | (rdp << 24) | (pf << 16) | (ps << 8) | source;

    //  The 0 as second parameter is a placeholder, don't know if it is viable in ExtendedFrame
    drivers::socketcan::CanId can_id(ID, 0, drivers::socketcan::FrameType::DATA, drivers::socketcan::ExtendedFrame);

    //  Setup Payload 
    std::array<uint8_t, 8> payload;
    packPayload(payload, left_speed, right_speed, msg_counter);

    //  Send the CAN message  
    try {
        //  Send the Message over CAN 
        // this->can_sender->send(&payload, 8, can_id);
        this->can_sender->send(payload.data(), payload.size(), can_id);
        msg_counter = static_cast<uint8_t>((msg_counter + 1) % 255);    //  modulo technique for rollover 
    } catch (drivers::socketcan::SocketCanTimeout& e) {
        BOOST_LOG_TRIVIAL(error) << "WheelController setSpeed timeout: " << std::hex << ID << std::dec
                                   << ", left speed=" << left_speed 
                                   << ", right speed=" << right_speed 
                                   << ", error: " << e.what();
        return false;
    }  
    return true;

}   //  setSpeed()

bool WheelController::isSetup() const { return this->setup_completed; }

void WheelController::update(const std::chrono::nanoseconds& timeout) {
    // Read a message from the CAN bus
    // TODO: Consider bus-level message filtering for efficiency
    try {
        uint8_t msg_buffer[8];

        drivers::socketcan::CanId msg_info = this->can_receiver->receive(&msg_buffer, timeout);

        //  Check if this isn't a standard CAN message, if so then it isn't a message applicable to us
        if (msg_info.frame_type() != drivers::socketcan::FrameType::DATA) { return; }   

        //  Check if this CAN message is 29-bit, J1939, Extended
        if (!msg_info.is_extended() ) { return; }

        // Turn the raw buffer into a vector
        std::vector msg(msg_buffer, msg_buffer + msg_info.length());

        this->handleCANMessage(msg, msg_info);
    }
    catch (drivers::socketcan::SocketCanTimeout& _) {} // Don't care if we don't receive a message
}   //  update()

void WheelController::handleCANMessage(const std::vector<uint8_t>& message, drivers::socketcan::CanId& info) {
    // Note: info can't be const because get_bus_time isn't const-qualified...

    //  Drop message if not addressed, this checks if the message is an Extended Frame message (29-bit CAN)
    if (!info.is_extended()) {
        //  Drop standard 11-bit messages, because we are J1939, 29-bit
        return;
    } 

    //  If there is no payload, just send a debug message saying so 
    if (message.empty()) {
        BOOST_LOG_TRIVIAL(debug) << "[" << info.get_bus_time() << "]: Message received for: " << std::hex
                                 << info.identifier() << std::dec << " with no payload";
    }   

    //  Grab the command from the CanId, command is not in the message anymore
    uint32_t full_id = info.identifier();
    uint32_t command = (full_id >> 8) & 0xFF;
 
    // Process the message
    switch (command) {
        //  Fix this let this handle the CAN messages received, STM_ECHO, GET_SPEED (TBD), + more if necessary
        case RoverCommands::ECHO_RESPONSE: this->handleEcho(message, info); break;
        case RoverCommands::GET_SPEED: this->handleGetSpeed(message, info); break;
        default:
            // Again, we are subscribing to all messages on the bus, no need to spam log with ignored messages
            break;
    }
}   //  handleCANMessage()

void WheelController::handleEcho(const std::vector<uint8_t>& message, drivers::socketcan::CanId& info) {
    (void)message;
    BOOST_LOG_TRIVIAL(debug) << "[" << info.get_bus_time() << "]: STM32 ECHO received " << std::hex
                             << info.identifier();
}   //  handleEcho()

void WheelController::handleGetSpeed(const std::vector<uint8_t>& message, drivers::socketcan::CanId& info) {
    if (message.size() != 4) {
        return;
    }
    uint16_t raw_left  = static_cast<uint16_t>((message[1] << 8) | message[0]); 
    uint16_t raw_right = static_cast<uint16_t>((message[3] << 8) | message[2]);

    //  Check for 0xFFFE or 0xFFFF
    if (raw_left >= 0xFFFE || raw_right >= 0xFFFE ) {
        BOOST_LOG_TRIVIAL(error) << "getSpeed Error: " << std::hex << info.identifier()
                            << ", left speed=" << raw_left 
                            << ", right speed=" << raw_right;
        return;
    }

    constexpr double SLOT_SCALE = 0.125;
    constexpr double SLOT_OFFSET = -4016.0;

    double left_rpm  = (static_cast<double>(raw_left) * SLOT_SCALE) + SLOT_OFFSET;
    double right_rpm = (static_cast<double>(raw_right) * SLOT_SCALE) + SLOT_OFFSET;

    //  Unit Conversion to rad/sec
    // constexpr double RPM_TO_RAD_SEC = (2.0 * M_PI) / 60.0;
    // double left_rad_sec  = left_rpm * RPM_TO_RAD_SEC;
    // double right_rad_sec = right_rpm * RPM_TO_RAD_SEC;


    //  Need to determine if we want RPM back or m/s, also how does the STM32 send the speed
    BOOST_LOG_TRIVIAL(debug) << "[" << info.get_bus_time() << "]: getSpeed received " << std::hex
                             << info.identifier() << std::dec << " left speed= " << left_rpm
                             << " RPM, right speed= " << right_rpm << " RPM";
}   //  handleGetSpeed()