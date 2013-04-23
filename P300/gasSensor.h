/*
 * Gas Sensor Round Robin Library
 *
 */
#ifndef gassensor_h
#define gassensor_h

// Num of reads for analogReadAverage
#define NUM_ANALOG_READS 10

#include <inttypes.h>

 
class GasSensor
{
  protected:
    int  pin; // Pin Number
    bool pinhigh; // Pin voltage > 1.2V
    int lval; // Last value
    
  public:
   // initialize gas sensor with pin (int)
   GasSensor(int);
   // read actual value
   uint16_t getValue();
   // read actual value
   int8_t getQuality();
   // loop action
   uint16_t loopAction();
};
#endif
