// c++ test.cpp modbuscrc.cpp gasSensor.cpp -o /tmp/test && /tmp/test

#ifndef ARDUINO
#include <fstream>
#include <iostream>
#include <iomanip>
#include "modbuscrc.h"
#include "gasSensor.h"

using namespace std;

ModbusCRC crc=ModbusCRC();
GasSensor gas =GasSensor(1,1);

uint8_t buf[] = {01,03,00,05,00,02,0xd4,0x0a};

void printhtmap()
{
    cout << "HTMAP: " << sizeof(GasSensor::htmap_avg) << " Bytes" << endl;
  cout << "°C  %";
  for (uint8_t hum=0; hum<HT_MAP_COUNT_HUM; hum++)
  {
    cout << setw(6) << (int)((hum*HT_MAP_DIV_HUM)+HT_MAP_MIN_HUM);
  }
  cout << endl;

  for (uint8_t temp=0; temp<HT_MAP_COUNT_TEMP; temp++)
  {
    cout << setw(3) << (int)((temp*HT_MAP_DIV_TEMP)+HT_MAP_MIN_TEMP) << "  ";
    for (uint8_t hum=0; hum<HT_MAP_COUNT_HUM; hum++)
    {
      cout << setw(6) << (int)gas.htmap_avg[hum][temp];
    }
    cout <<endl;
  }
}

int main()
{
   // CRC Test
   //=================
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
  
  // gasSensor Test
  //================
  ifstream is("dump");
  if (is)
  {
    int temp1, hum1;
    char lq = 0;
    while (is >> gas.millis_val >> temp1 >> hum1 >> gas.analog_val)
    {
      gas.loopAction(temp1, hum1);
      if (lq!=gas.getQuality())
      {
        lq=gas.getQuality();
//        if ( (gas.millis_val>1743459) && (gas.millis_val<1743714))
        {
          cout << gas.millis_val << " " << temp1 << " " << hum1 << " " << gas.analog_val <<  " " << (int)gas.getQuality();
          if (lq<80) cout << " STOP" << endl;
            else cout << " OK" << endl;
        }
      }
    }
    printhtmap();
  }
  
  return 0;
}

#endif
