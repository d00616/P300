/*
 * Gas Sensor Round Robin Library
 *
 */
#include "gasSensor.h"
#ifdef ARDUINO
  #include <Arduino.h>
#else
  // Arduino emulation
  uint32_t GasSensor::millis() { return millis_val*1000; }
  uint16_t GasSensor::analogRead(int pin) { return analog_val; }
#endif

// Constructor
GasSensor::GasSensor(char indexnumber, int pinnumber)
{
     pin = pinnumber;
#ifdef ARDUINO
     analogReadRes(16);
     analogReadAveraging(NUM_ANALOG_READS);
     
     // Check PIN voltage
     pinMode(pin, INPUT);
#else
    millis_val=0;
    analog_val=0;
#endif
     // Stop reading analog values to protect pin
     pinhigh=true;
     
     lval=0;
     quality=100;
     for (char i=0; i<QUALITY_HISTORY;i++) quality_history[i]=100;
     quality_history_pos=0;

     mapresettime=MAP_RESET_TIME;
 
     for (uint8_t hum=0; hum<HT_MAP_COUNT_HUM; hum++)
     {
       for (uint8_t temp=0; temp<HT_MAP_COUNT_TEMP; temp++)
       {
         htmap[hum][temp]=65535;
         htmap_time[hum][temp]=65535;
       }
     }

}

void GasSensor::sethtmap(char temp, char hum, uint16_t val)
{
  char hpos = (hum-HT_MAP_MIN_HUM)/HT_MAP_DIV_HUM;
  if ((hpos<0) || (hpos>=HT_MAP_COUNT_HUM)) return;
  char tpos = (temp-HT_MAP_MIN_TEMP)/HT_MAP_DIV_TEMP;
  if ((tpos<0) || (tpos>=HT_MAP_COUNT_TEMP)) return;

  if (htmap_time[hpos][tpos]>val)
  {
      htmap_time[hpos][tpos]=val;  
      if (htmap[hpos][tpos]>val) htmap[hpos][tpos]=val;
  }
}

uint16_t GasSensor::getValue()
{
  if (lval==0) return analogRead(pin); 
  return lval;
}

int8_t GasSensor::getQuality()
{
  return quality;
}

int8_t GasSensor::getQualityHistoryDelta()
{
  int8_t min=100;
  int8_t max=0;
  for (char i=0;i<QUALITY_HISTORY;i++)
  {
     if (min>quality_history[i]) min=quality_history[i];
     if (max<quality_history[i]) max=quality_history[i];
  }
  
  // return a value if quality are increasing
  if ((quality==min) && (quality<max)) return min-max;
  if ((quality==max) && (quality>min)) return min-max;
  return 0;
}

uint16_t GasSensor::loopAction(char temp, char humidity)
{
  // Time calculation
  uint32_t start_millis = millis();
  if (start_millis>lmillis)
  {
    mapresettime=mapresettime-((start_millis-lmillis)/1000);      
    if (mapresettime<=0)
    {
      mapresettime = MAP_RESET_TIME;
    
     // htmap_time -> htmap
     for (uint8_t hum=0; hum<HT_MAP_COUNT_HUM; hum++)
     {
       for (uint8_t temp=0; temp<HT_MAP_COUNT_TEMP; temp++)
       {
         if (htmap_time[hum][temp]<65535)
         {
           htmap[hum][temp]=(htmap[hum][temp]+htmap_time[hum][temp])/2;
           htmap_time[hum][temp]=65535;
         }
       }
     }
    }
  }
  lmillis=start_millis;
  
  // Pin protection
#ifdef ARDUINO
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
#endif

  // Read value
  int tmp=analogRead(pin);

  // Pin protection II Value near maximum (1.2V)?
  if (tmp>65000)
  {
      pinhigh=true;
  }
  
  // Burn in time?
  if ( (lval==0) && (millis()<SENSOR_GAS_PREHEAT_TIME))
  {
    ltmp = temp;
    lhum = humidity;
    return tmp;
  }
 
  // Calculation
  char hpos = (humidity-HT_MAP_MIN_HUM)/HT_MAP_DIV_HUM;
  if (hpos<0) hpos==0;
  if (hpos>=HT_MAP_COUNT_HUM) hpos=HT_MAP_COUNT_HUM-1;
  char tpos = (temp-HT_MAP_MIN_TEMP)/HT_MAP_DIV_TEMP;
  if (tpos<0) tpos=0;
  if (tpos>=HT_MAP_COUNT_TEMP) tpos=HT_MAP_COUNT_TEMP-1;
  
  // New value?
  if ((htmap[hpos][tpos]==65536) && (quality<100))
  {
    sethtmap(temp,humidity,(int)((float)lval/(float)quality)*tmp);
  }

  lval = tmp;
  sethtmap(temp,humidity,tmp);
  

  quality = ( (float)htmap[hpos][tpos]/(float)lval)*100;
  ltmp = temp;
  lhum = humidity;
  
  // Store quality history
  quality_history[quality_history_pos]=quality;
  quality_history_pos++;
  if (quality_history_pos>=QUALITY_HISTORY) quality_history_pos=0;
  
  return lval;
}
