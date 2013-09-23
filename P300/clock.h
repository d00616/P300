/*
 * Gas Sensor Round Robin Library
 *
 */
#ifndef clock_h
#define clock_h

#include <inttypes.h>

 
class Clock
{
  protected:
    uint8_t hour,minute,second,weekday;
    bool stopincrement;
    
  public:
   // initialize gas sensor with pin (int)
   Clock();
   
   void setTime(uint8_t,uint8_t,uint8_t);
   void secondAction();
   
   uint8_t getHour();
   uint8_t getMinute();
   uint8_t getSecond();
   uint8_t getWeekday();
};
#endif
