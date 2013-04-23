/*
 * CRC Library
 *
 * 
 */
#include "modbuscrc.h"

// Constructor
ModbusCRC::ModbusCRC()
{
     mask = MODBUS_MASK;
}

// Reset crc calculation
void ModbusCRC::clearCRC()
{
   crc = 0xffff;
}

// Calculate CRC for one byte
void ModbusCRC::add_byte(uint8_t byte)
{
  crc = crc ^ byte;
  for (int i=0; i<8; i++)
  {
    if (crc & 1)
    {
       crc = crc >> 1;
       crc = crc ^ mask;
    }
       else
    {
       crc = crc >>1;
    }
  }
}

// Compare CRC with bytes
bool ModbusCRC::compareCRC(uint8_t firstbyte, uint8_t secondbyte)
{
  if ( (firstbyte == (crc & 0xff) ) && (secondbyte == (crc >> 8) )) return true;
  return false;
}

// Swap L<->H
uint16_t ModbusCRC::getCRC()
{
   return (crc >> 8) | (crc << 8);
}
