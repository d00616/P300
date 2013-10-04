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
    unsigned long updatetime;
    int8_t hour,minute,second,weekday;
    bool stopincrement;
    
  public:
   // initialize gas sensor with pin (int)
   Clock();
   
   void setTime(uint8_t,uint8_t,uint8_t);
   void secondAction();
   
   int8_t getHour();
   int8_t getMinute();
   int8_t getSecond();
   int8_t getWeekday();
};
#endif
