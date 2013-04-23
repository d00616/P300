// c++ test.cpp modbuscrc.cpp -o /tmp/test && /tmp/test

#ifndef ARDUINO
#include <iostream>
#include "modbuscrc.h"
using namespace std;

ModbusCRC crc=ModbusCRC();
uint8_t buf[] = {01,03,00,05,00,02,0xd4,0x0a};

int main()
{
  cout << "Compare CRC: 01 03 00 05 00 02 :: D4 0A -> ";
  crc.clearCRC();
  crc.add_byte(0x01);
  crc.add_byte(0x03);
  crc.add_byte(0x00);
  crc.add_byte(0x05);
  crc.add_byte(0x00);
  crc.add_byte(0x02);
  if ( crc.compareCRC(0xd4, 0x0a) )
  {
    cout << "OK" << endl;
  }
    else
  {
    cout << "ERR" << endl;
  }
  
  return 0;
}

#endif
