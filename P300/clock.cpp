#include "clock.h"

// Constructor
Clock::Clock()
{
  hour=-1;
  minute=-1;
  second=-1;
  weekday=-1;
  stopincrement=false;
}

void Clock::secondAction()
{
  if ((weekday>=0) && (!stopincrement))
  {
    if (second++>59)
    {
      second=0;
      if (hour++>23)
      {
        hour=0;
        if (weekday++>7)
        {
          weekday=1;
        }
      }
    }
  }
  stopincrement=false;
}

void Clock::setTime(uint8_t hour,uint8_t minute,uint8_t weekday)
{
  // new time
  if (weekday==-1)
  {
    this->hour=hour;
    this->minute=minute;
    this->weekday=weekday;
    this->second=30;
  }
    else
  {
    // myclock is in future
    if ((this->minute>minute) || (this->hour>hour)) stopincrement=true;
    // myclock is behind
    if ((this->minute<minute) || (this->hour<hour)) secondAction();
  }
}
   
uint8_t Clock::getHour() { return hour; }
uint8_t Clock::getMinute() { return minute; }
uint8_t Clock::getSecond() { return second; }
uint8_t Clock::getWeekday() { return weekday; }

