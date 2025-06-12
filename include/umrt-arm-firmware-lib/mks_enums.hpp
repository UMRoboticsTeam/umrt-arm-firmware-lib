//
// Created by noah on 11/06/25.
//
/**
* @file
* Enums for interacting with commands described in @ref MKS_COMMANDS.hpp.
* These are separated because MKS_COMMANDS.hpp is parsed by MksTest.py to determine the command bytes it uses for its
* implementation.
*/

#ifndef UMRT_ARM_FIRMWARE_LIB_MKS_ENUMS_HPP
#define UMRT_ARM_FIRMWARE_LIB_MKS_ENUMS_HPP

#include <string>

#include "MKS_COMMANDS.hpp"

/** Status code for the response to move commands.
 * Used by @ref SEND_STEP, @ref SEEK_POS_BY_STEPS, @ref SEND_ANGLE, and @ref SEEK_POS_BY_ANGLE.
 */
enum MksMoveResponse : uint8_t {
    /** Movement failed. */
    FAILED = 0,

    /** The motor is moving. */
    MOVING = 1,

    /** The motor has reached the target position. */
    COMPLETED = 2,

    /** An end limit has been reached. */
    LIMIT_REACHED = 3
};

/**
* Converts an @ref MksMoveResponse to its string representation.
* @param status response status to lookup
*/
inline std::string to_string_mks_move_response(const MksMoveResponse status) {
    switch (status) {
        case MksMoveResponse::FAILED: return "FAILED";
        case MksMoveResponse::MOVING: return "MOVING";
        case MksMoveResponse::COMPLETED: return "COMPLETED";
        case MksMoveResponse::LIMIT_REACHED: return "LIMIT_REACHED";
    }
    throw std::logic_error("MksMoveResponse passed with invalid value: " + std::to_string(static_cast<uint8_t>(status)));
}

#endif //UMRT_ARM_FIRMWARE_LIB_MKS_ENUMS_HPP
