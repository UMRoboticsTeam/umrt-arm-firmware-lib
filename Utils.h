/**
 * @file
 * A collection of helper functions related to encoding/decoding data for communication over a Firmata link.
 */

#ifndef COMMUNICATION_MASTER_EXAMPLE_UTILS_H
#define COMMUNICATION_MASTER_EXAMPLE_UTILS_H

#include <cstdint>
#include <string>
#include <vector>

/**
 * Packs a 32-bit integer into a vector of 8-bit integers, in little-endian format.
 *
 * E.g. for `0xDEAD_BEEF`:
 * ```
 * input:        1101 1110 1010 1101 1011 1110 1110 1111
 * bit grouping: 3333 3333 2222 2222 1111 1111 0000 0000
 * output:       { 0xEF, 0xBE, 0xAD, 0xDE }
 * ```
 *
 * @param integer the integer to pack
 * @return the byte representation of `integer`
 */
inline std::vector<uint8_t> pack_32(const uint32_t integer) {
    return {
            static_cast<unsigned char>(integer & 0xFF),       // bits [7, 0]
            static_cast<unsigned char>(integer >> 8 & 0xFF),  // bits [15, 8]
            static_cast<unsigned char>(integer >> 16 & 0xFF), // bits [23, 16]
            static_cast<unsigned char>(integer >> 24 & 0xFF)  // bits [31, 24]
    };
}

/**
 * Packs a 16-bit integer into a vector of 8-bit integers, in little-endian format.
 *
 * E.g. for `0xBEEF`:
 * ```
 * input:        1011 1110 1110 1111
 * bit grouping: 1111 1111 0000 0000
 * output:       { 0xEF, 0xBE }
 * ```
 *
 * @param integer the integer to pack
 * @return the byte representation of `integer`
 */
inline std::vector<uint8_t> pack_16(const uint16_t integer) {
    return {
            static_cast<unsigned char>(integer & 0xFF),     // bits [7, 0]
            static_cast<unsigned char>(integer >> 8 & 0xFF) // bits [15, 8]
    };
}

/**
 * Converts a packed byte vector to the 7-bit packets Firmata receives.
 * Useful for checking `a == decode_32(firmatify_32(pack_32(a)))`
 *
 * E.g. for `{ 0xEF, 0xBE, 0xAD, 0xDE }`:
 * ```
 * input = { 1101 1110, 1010 1101, 1011 1110, 1110 1111 }
 * firmatified = { 0101 1110, 0000 0001, 0010 1101, 0000 0001, 0011 1110, 0000 0001, 0110 1111, 0000 0001 }
 *             = { 0x5E, 0x01, 0x2D, 0x01, 0x3E, 0x01, 0x6F, 0x01 }
 * ```
 *
 * @param pack iterator to the byte vector subset to convert
 * @return the firmatified representation of `pack`
 */
inline std::vector<uint8_t> firmatify_32(const std::vector<uint8_t>::const_iterator& pack) {
    return {
            static_cast<unsigned char>(pack[0] & 0x7F),
            static_cast<unsigned char>((pack[0] & 0x80) >> 7),
            static_cast<unsigned char>(pack[1] & 0x7F),
            static_cast<unsigned char>((pack[1] & 0x80) >> 7),
            static_cast<unsigned char>(pack[2] & 0x7F),
            static_cast<unsigned char>((pack[2] & 0x80) >> 7),
            static_cast<unsigned char>(pack[3] & 0x7F),
            static_cast<unsigned char>((pack[3] & 0x80) >> 7),
    };
}

/**
 * Calls firmatify_32(const std::vector<uint8_t>::const_iterator&) with `pack.cbegin()`
 *
 * @param pack the byte vector to convert
 * @return the firmatified representation of `pack`
 */
inline std::vector<uint8_t> firmatify_32(const std::vector<uint8_t>& pack) { return firmatify_32(pack.cbegin()); }

/**
 * Converts a packed byte vector to the 7-bit packets Firmata receives.
 * Useful for checking `a == decode_16(firmatify_16(pack_16(a)))`
 *
 * e.g.  for `{ 0xEF, 0xBE }`:
 * ```
 * input = { 1101 1110, 1010 1101 }
 * firmatified = { 0101 1110, 0000 0001, 0010 1101, 0000 0001 }
 *             = { 0x5E, 0x01, 0x2D, 0x01 }
 * ```
 *
 * @param pack iterator to the byte vector subset to convert
 * @return the firmatified representation of `pack`
 */
inline std::vector<uint8_t> firmatify_16(const std::vector<uint8_t>::const_iterator& pack) {
    return {
            static_cast<unsigned char>(pack[0] & 0x7F),
            static_cast<unsigned char>((pack[0] & 0x80) >> 7),
            static_cast<unsigned char>(pack[1] & 0x7F),
            static_cast<unsigned char>((pack[1] & 0x80) >> 7),
            static_cast<unsigned char>(pack[2] & 0x7F),
            static_cast<unsigned char>((pack[2] & 0x80) >> 7),
            static_cast<unsigned char>(pack[3] & 0x7F),
            static_cast<unsigned char>((pack[3] & 0x80) >> 7),
    };
}

/**
 * Calls firmatify_16(const std::vector<uint8_t>::const_iterator&) with `pack.cbegin()`
 *
 * @param pack the byte vector to convert
 * @return the firmatified representation of `pack`
 */
inline std::vector<uint8_t> firmatify_16(const std::vector<uint8_t>& pack) { return firmatify_16(pack.cbegin()); }

/**
 * Converts an 8-bit integer to the two 7-bit packets Firmata uses.
 *
 * E.g.  for `0xBE`:
 * input = 1010 1101
 * firmatified = { 0010 1101, 0000 0001 }
 *             = { 0x2D, 0x01 }
 *
 * @param val the integer to convert
 * @return the firmatified representation of `val`
 */
inline std::vector<uint8_t> firmatify_8(const uint8_t val) {
    return { static_cast<unsigned char>(val & 0x7F), static_cast<unsigned char>((val & 0x80) >> 7) };
}

/**
 * Reconstructs a little-endian byte vector into a 32-bit integer.
 *
 * @param data iterator to the byte vector subset to convert
 * @return a 32-bit integer
 */
inline uint32_t decode_32(const std::vector<uint8_t>::const_iterator& data) {
    return data[0] | data[1] << 8 | data[2] << 16 | data[3] << 24;
}

/**
 * Calls decode_32(const std::vector<uint8_t>::const_iterator&) with `data.cbegin()`
 *
 * @param data the byte vector to convert
 * @return a 32-bit integer
 */
inline uint32_t decode_32(const std::vector<uint8_t>& data) { return decode_32(data.cbegin()); }

/**
 * Reconstructs a little-endian byte vector into a 16-bit integer.
 *
 * @param data iterator to the byte vector subset to convert
 * @return a 16-bit integer
 */
inline uint16_t decode_16(const std::vector<uint8_t>::const_iterator& data) {
    return data[0] | data[1] << 8;
}

/**
 * Calls decode_16(const std::vector<uint8_t>::const_iterator&) with `data.cbegin()`
 *
 * @param data the byte vector to convert
 * @return a 16-bit integer
 */
inline uint16_t decode_16(const std::vector<uint8_t>& data) { return decode_16(data.cbegin()); }

/**
 * Packs an std::string into a vector of 8-bit integers, in little-endian format.
 *
 * @param str the std::string to pack
 * @return the byte representation of `str`
 */
inline std::vector<uint8_t> encode_string(const std::string& str) {
    return { str.cbegin(), str.cend() };
}

/**
 * Decodes a little-endian byte vector into an std::string.
 *
 * @param data the byte vector to convert
 * @return an std::string
 */
inline std::string decode_string(const std::vector<uint8_t>& data) {
    return { data.cbegin(), data.cend() };
}

#endif //COMMUNICATION_MASTER_EXAMPLE_UTILS_H
