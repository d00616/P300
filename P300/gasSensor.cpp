/*
 * Gas Sensor Round Robin Library
 *
 */
#include <Arduino.h>
#include "gasSensor.h"

// Constructor
GasSensor::GasSensor(int pinnumber)
{
     pin = pinnumber;
     analogReadRes(16);
     analogReadAveraging(NUM_ANALOG_READS);
     
     // Check PIN voltage
     pinMode(pin, INPUT);
     // Stop reading analog values to protect pin
     pinhigh=true;
}

uint16_t GasSensor::getValue()
{
  return lval;
}

int8_t GasSensor::getQuality()
{
  return 50;
}


uint16_t GasSensor::loopAction()
{
  // Pin protection
  if (pinhigh)
  {
     if (!digitalRead(pin))
     {
      pinhigh=false;
     }
       else
     {
        return 65536;
     }
  }  
  
  lval = analogRead(pin);
  
  // Pin protection II Value near maximum (1.2V)?
  if (lval>65000)
  {
      pinhigh=true;
  }
  
  return lval;
}
