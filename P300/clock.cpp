#include "clock.h"
#include "Arduino.h"

// Constructor
Clock::Clock()
{
  hour=-1;
  minute=-1;
  second=-1;
  weekday=-1;
  stopincrement=false;
  updatetime=millis();
}

void Clock::secondAction()
{
  updatetime=millis()+970; // 30 millis before next call
  if ((weekday>=0) && (!stopincrement))
  {
    if (second++>59)
    {
      second=0;
      if (minute++>59)
      {
        minute=0;
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
  }
  stopincrement=false;
}

void Clock::setTime(uint8_t p_hour,uint8_t p_minute,uint8_t p_weekday)
{
  int nclock=p_weekday*24*7+p_hour*24+p_minute;
  int oclock=this->weekday*24*7+this->hour*24+this->minute;

  // new time
  if (((nclock-oclock)>2) || ((nclock-oclock)<-2))
  {
    this->hour=p_hour;
    this->minute=p_minute;
    this->weekday=p_weekday;
    if (this->second==-1) this->second=30;
  }
    else
  {
    // myclock is in future
    if ((this->minute>p_minute) || (this->hour>p_hour)) stopincrement=true;
    // myclock is behind
    if ((this->minute<p_minute) || (this->hour<p_hour)) secondAction();
  }
}
   
int8_t Clock::getSecond()
{
  // add a second if second==59 and next increment in 30 milliseconds
  if ( (second==59) && (millis()>=updatetime))
  {
    secondAction();
    stopincrement=true;
  }
  return second;
}
int8_t Clock::getHour() { return hour; }
int8_t Clock::getMinute() { return minute; }
int8_t Clock::getWeekday() { return weekday; }

