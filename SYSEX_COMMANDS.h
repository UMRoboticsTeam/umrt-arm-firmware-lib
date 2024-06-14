#ifndef SYSEX_COMMANDS_H
#define SYSEX_COMMANDS_H

#include <cstdint>

// TODO: Consider adding a keep-alive
// Note: Only allowed to use 0x00-0x0F, beyond that would have to nest commands
// Multi-byte numbers are always transmitted in little-endian
enum SysexCommands : uint8_t {
  // Respond back with payload
  // In:  [ payload (anything) ]
  // Out: [ payload (anything) ]
  ARDUINO_ECHO = 0x00,

  // Set the speed of a motor
  // In:  [ motor_id (uint8_t), speed (int16_t)]
  // Out: [ motor_id (uint8_t), speed (int16_t)]
  SET_SPEED = 0x01,

  // Get the current speed of a motor
  // In:  [ motor_id (uint8_t) ]
  // Out: [ motor_id (uint8_t), speed (int16_t)]
  GET_SPEED = 0x02
};

#endif
