//  Credits: https://github.com/II43/Crc8_J1850

#ifndef CRC8_J1850
#define CRC8_J1850

typedef unsigned char uint8;

uint8 Crc8(const uint8* data, unsigned int len, uint8 startvalue); 

#endif