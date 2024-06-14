//
// Created by Noah on 2024-05-31.
//

#ifndef COMMUNICATION_MASTER_EXAMPLE_UTILS_H
#define COMMUNICATION_MASTER_EXAMPLE_UTILS_H

#include <cstdint>
#include <string>
#include <vector>

// Packs a 32-bit integer into a vector of 8-bit integers, in little-endian format.
//
// e.g. for 0xDEAD_BEEF:
// 1101 1110 1010 1101 1011 1110 1110 1111
// 3333 3333 2222 2222 1111 1111 0000 0000
// packed = [ 0xEF, 0xBE, 0xAD, 0xDE ]
inline std::vector<uint8_t> pack_32(const uint32_t integer) {
    return std::vector<uint8_t>({
            static_cast<unsigned char>(integer & 0xFF),       // bits [7, 0]
            static_cast<unsigned char>(integer >> 8 & 0xFF),  // bits [15, 8]
            static_cast<unsigned char>(integer >> 16 & 0xFF), // bits [23, 16]
            static_cast<unsigned char>(integer >> 24 & 0xFF)  // bits [31, 24]
    });
}

// Packs a 16-bit integer into a vector of 8-bit integers, in little-endian format.
//
// e.g. for 0xBEEF:
// 1011 1110 1110 1111
// 1111 1111 0000 0000
// packed = [ 0xEF, 0xBE ]
inline std::vector<uint8_t> pack_16(const uint16_t integer) {
    return std::vector<uint8_t>({
            static_cast<unsigned char>(integer & 0xFF),     // bits [7, 0]
            static_cast<unsigned char>(integer >> 8 & 0xFF) // bits [15, 8]
    });
}

// Convert a packed byte vector to the 7-bit packets Firmata receives.
// Useful for checking a == decode_32(firmatify_32(pack_32(a)))
//
// e.g.  for [0xEF, 0xBE, 0xAD, 0xDE]:
// [1101 1110, 1010 1101, 1011 1110, 1110 1111]
// firmatified = [ 0101 1110, 0000 0001, 0010 1101, 0000 0001, 0011 1110, 0000 0001, 0110 1111, 0000 0001 ]
//             = [ 0x5E, 0x01, 0x2D, 0x01, 0x3E, 0x01, 0x6F, 0x01]
inline std::vector<uint8_t> firmatify_32(const std::vector<uint8_t>::const_iterator& pack) {

    return std::vector<uint8_t>({
            static_cast<unsigned char>(pack[0] & 0x7F),
            static_cast<unsigned char>((pack[0] & 0x80) >> 7),
            static_cast<unsigned char>(pack[1] & 0x7F),
            static_cast<unsigned char>((pack[1] & 0x80) >> 7),
            static_cast<unsigned char>(pack[2] & 0x7F),
            static_cast<unsigned char>((pack[2] & 0x80) >> 7),
            static_cast<unsigned char>(pack[3] & 0x7F),
            static_cast<unsigned char>((pack[3] & 0x80) >> 7),
    });
}

inline std::vector<uint8_t> firmatify_32(const std::vector<uint8_t>& pack) { return firmatify_32(pack.cbegin()); }

// Convert a packed byte vector to the 7-bit packets Firmata receives.
// Useful for checking a == decode_16(firmatify_16(pack_16(a)))
//
// e.g.  for [0xEF, 0xBE]:
// [1101 1110, 1010 1101]
// firmatified = [ 0101 1110, 0000 0001, 0010 1101, 0000 0001 ]
//             = [ 0x5E, 0x01, 0x2D, 0x01]
inline std::vector<uint8_t> firmatify_16(const std::vector<uint8_t>::const_iterator& pack) {
    return std::vector<uint8_t>({
            static_cast<unsigned char>(pack[0] & 0x7F),
            static_cast<unsigned char>((pack[0] & 0x80) >> 7),
            static_cast<unsigned char>(pack[1] & 0x7F),
            static_cast<unsigned char>((pack[1] & 0x80) >> 7),
            static_cast<unsigned char>(pack[2] & 0x7F),
            static_cast<unsigned char>((pack[2] & 0x80) >> 7),
            static_cast<unsigned char>(pack[3] & 0x7F),
            static_cast<unsigned char>((pack[3] & 0x80) >> 7),
    });
}

inline std::vector<uint8_t> firmatify_16(const std::vector<uint8_t>& pack) { return firmatify_16(pack.cbegin()); }

// Convert a packed byte vector to the 7-bit packets Firmata receives.
// Useful for checking a == decode_8(firmatify_8(a))
//
// e.g.  for [0xBE]:
// [1010 1101]
// firmatified = [ 0010 1101, 0000 0001 ]
//             = [ 0x2D, 0x01]
inline std::vector<uint8_t> firmatify_8(const uint8_t val) {
    return std::vector<uint8_t>(val & 0x7F, (val & 0x80) >> 7);
}

// Decode Firmata 7-bit packets into a 32-bit integer
// See firmatify_32 for an explanation of what Firmata does to packets
inline uint32_t decode_32(const std::vector<uint8_t>::const_iterator& data) {
    return data[0] | data[1] << 7 | (data[2] | data[3] << 7) << 8 | (data[4] | data[5] << 7) << 16 | (data[6] | data[7] << 7) << 24;
}

inline uint32_t decode_32(const std::vector<uint8_t>& data) { return decode_32(data.cbegin()); }

// Decode Firmata 7-bit packets into a 16-bit integer
// See firmatify_16 for an explanation of what Firmata does to packets
inline uint16_t decode_16(const std::vector<uint8_t>::const_iterator& data) {
    return data[0] | data[1] << 7 | (data[2] | data[3] << 7) << 8;
}

inline uint16_t decode_16(const std::vector<uint8_t>& data) { return decode_16(data.cbegin()); }

// Decode Firmata 7-bit packets into an 8-bit integer
// See firmatify_8 for an explanation of what Firmata does to packets
inline uint16_t decode_8(const std::vector<uint8_t>::const_iterator& data) { return data[0] | data[1] << 7; }

inline uint8_t decode_8(const std::vector<uint8_t>& data) { return decode_8(data.cbegin()); }

// Decode a Firmata 7-bit packet stream into an std::string
std::string decode_string(const std::vector<uint8_t>& data) {
    std::string s;

    // Need at least 2 elements to do anything
    if (data.size() <= 1) { return s; }

    // Since we consume 2 elements at a time, we need to iterate until the second-last element
    // Note: data.cend() - 1 is safe because we know data.size() >= 2
    std::vector<uint8_t>::const_iterator end = data.cend() - 1;
    for (std::vector<uint8_t>::const_iterator i = data.cbegin(); i != end; i += 2) {
        s.append(static_cast<char>(decode_8(i)));
    }

    return s;
}

#endif //COMMUNICATION_MASTER_EXAMPLE_UTILS_H
