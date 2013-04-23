/*
 * Modbus CRC Library
 *
 */
 
#ifndef modbuscrc_h
#define modbuscrc_h

#include <inttypes.h>

#define MODBUS_MASK 0xA001
 
class ModbusCRC
{
  protected:
    uint16_t  mask;
    uint16_t  crc;
    
  public:
   ModbusCRC();
   void clearCRC();
   void add_byte(uint8_t);
   bool compareCRC(uint8_t, uint8_t);
   uint16_t getCRC();
};
#endif

