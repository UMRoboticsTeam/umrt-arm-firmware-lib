//
// Created by Niko Christie on 2025-07-01.
//

#include <boost/log/trivial.hpp>

#include <ros2_socketcan/socket_can_receiver.hpp>
#include <ros2_socketcan/socket_can_sender.hpp>

#include <numeric>

#include "ROVER_COMMANDS.hpp"
#include "wheel_controller.hpp"
#include "utils.hpp"
#include <cmath>

uint8_t msg_counter = 0;
// uint8_t checksum(uint16_t driver_id, const std::vector<uint8_t>& payload);

WheelController::WheelController() {

    BOOST_LOG_TRIVIAL(trace) << "WheelController construction begun.";

    this->can_receiver = std::make_unique<drivers::socketcan::SocketCanReceiver>(can_interface);
    this->can_sender = std::make_unique<drivers::socketcan::SocketCanSender>(can_interface);

    //TODO: Write norm_factor as microstepping factor to the driver

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

bool WheelController::setSpeed(const int16_t left_speed, const int16_t right_speed){

    if (!isSetup()) { return false; }

    //  Setup CAN ID
    uint32_t priority = 1;
    uint32_t rdp = 0;
    uint32_t pf = 0xFF; 
    uint32_t ps = RoverCommands::SET_SPEED; //  Command for Set Wheel Speed
    uint32_t source = 0x80  //  Source Address for Jetson Xavier AGX placeholder - ea

    uint32_t ID = (priority << 26) | (rdp << 24) | (pf << 16) | (ps << 8) | source;

    //  The 0 as second parameter is a placeholder, don't know if it is viable in ExtendedFrame
    drivers::socketcan::CanId can_id(ID, 0, drivers::socketcan::FrameType::DATA, drivers::socketcan::ExtendedFrame);

    //  Setup Payload 
    std::vector<uint8_t> payload;
    //  Make sure to create a counter 
    packPayload(payload, left_speed, right_speed, msg_counter);
    //  Non-vector method 
    // uint8_t payload[8] = {0};
    // // Left Wheel (Bytes 0-1)
    // payload[0] = left_speed & 0xFF;
    // payload[1] = (left_speed >> 8) & 0xFF;
    // // Right Wheel (Bytes 2-3)
    // payload[2] = right_speed & 0xFF;
    // payload[3] = (right_speed >> 8) & 0xFF;

    //  Send the CAN message  
    try {
        //  Send the Message over CAN 
        // this->can_sender->send(&payload, 8, can_id);
        this->can_sender->send(payload.data(), payload.size(), can_id);
        msg_counter++;
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
        if (msg_info.id_type() != drivers::socketcan::IdType::EXTENDED) { return; }

        // Turn the raw buffer into a vector
        std::vector msg(msg_buffer, msg_buffer + msg_info.length());

        this->handleCanMessage(msg, msg_info);
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

    //  If there is no payload, just send an Error message saying so 
    if (message.empty()) {
        BOOST_LOG_TRIVIAL(error) << "[" << info.get_bus_time() << "]: Message received for: " << std::hex
                                 << info.identifier() << std::dec << " with no payload";
    }   

    //  Grab the command from the CanId, command is not in the message anymore
    uint32_t full_id = info.identifier();
    uint32_t command = (full_id >> 8) & 0xFF;
 
    // Process the message
    switch (command) {
        //  Fix this let this handle the CAN messages received, STM_ECHO, GET_SPEED (TBD), + more if necessary
        case RoverCommands::STM_ECHO: this->handleEcho(message, info); break;
        case RoverCommands::GET_SPEED: this->handleGetSpeed(message, info); break;
        case RoverCommands::EMERGENCY_STOP: this->handleEStop(message, info); break;
        default:
            // Again, we are subscribing to all messages on the bus, no need to spam log with ignored messages
            break;
    }
}   //  handleCANMessage()

void WheelController::handleEcho(const std::vector<uint8_t>& message, drivers::socketcan::CanId& info) {
    BOOST_LOG_TRIVIAL(debug) << "[" << info.get_bus_time() << "]: STM32 ECHO received " << std::hex
                             << info.identifier();
}

void WheelController::handleGetSpeed(const std::vector<uint8_t>& message, drivers::socketcan::CanId& info) {
    BOOST_LOG_TRIVIAL(debug) << "[" << info.get_bus_time() << "]: Get Speed received " << std::hex
                             << info.identifier();
}

void WheelController::handleEStop(const std::vector<uint8_t>& message, drivers::socketcan::CanId& info) {
    BOOST_LOG_TRIVIAL(debug) << "[" << info.get_bus_time() << "]: Emergency Stop received " << std::hex
                             << info.identifier();
}

uint8_t WheelController::checksum(const std::vector<uint8_t>& payload) {
    //  CRC8 Checksum
    uint8_t crc = 0x00; // Standard J1939 start
    // We only calculate over the first 7 bytes
    for (size_t i = 0; i < 7; ++i) {
        crc ^= payload[i];
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x1D;
            }
            else {
                crc <<= 1;
            }
        }
    }
    return crc;
}   //  checksum()

void WheelController::packPayload(std::vector<uint8_t>& payload, const int16_t left_speed, const int16_t right_speed, uint8_t counter) {
    //  Clear and resize to exact size
    payload.assign(8, 0x00);

    //  Pack Speed Data (Bytes 0-3)
    payload[0] = static_cast<uint8_t>(left_speed & 0xFF);
    payload[1] = static_cast<uint8_t>((left_speed >> 8) & 0xFF);
    payload[2] = static_cast<uint8_t>(right_speed & 0xFF);
    payload[3] = static_cast<uint8_t>((right_speed >> 8) & 0xFF);

    //  Bytes 4-5 stay 0x00 - for now 

    //  Message Counter (Byte 6)
    payload[6] = counter;

    //  Checksum (Byte 7)
    payload[7] = checksum(payload);
}   //  packPayload()