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

     // Reset htmap
     mapresettime=MAP_RESET_TIME;
     for (uint8_t hum=0; hum<HT_MAP_COUNT_HUM; hum++)
     {
       for (uint8_t temp=0; temp<HT_MAP_COUNT_TEMP; temp++)
       {
         htmap_avg[hum][temp]=65535;
         htmap_max[hum][temp]=65535;
       }
     }

}

// Write a value to htmap
void GasSensor::sethtmap(char temp, char hum, uint16_t val)
{
  char hpos = (hum-HT_MAP_MIN_HUM)/HT_MAP_DIV_HUM;
  if ((hpos<0) || (hpos>=HT_MAP_COUNT_HUM)) return;
  char tpos = (temp-HT_MAP_MIN_TEMP)/HT_MAP_DIV_TEMP;
  if ((tpos<0) || (tpos>=HT_MAP_COUNT_TEMP)) return;

  // höchsten Wert für 100% finden
  if (val<htmap_max[hpos][tpos])
  {
      htmap_max[hpos][tpos]=val;
      
      // new 100%?
      if (val<htmap_avg[hpos][tpos]) htmap_avg[hpos][tpos]=val; 
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
  int avg=0;
  for (char i=0;i<QUALITY_HISTORY;i++)
  {
    avg=avg+quality_history[i];
  }
  avg=avg/QUALITY_HISTORY;
  return quality-avg;
}

uint16_t GasSensor::loopAction(char temp, char humidity)
{
  // Quality calculation after MAP_RESET_TIME
  uint32_t start_millis = millis();
  if (start_millis>lmillis)
  {
    mapresettime=mapresettime-((start_millis-lmillis)/1000);      
    if (mapresettime<=0)
    {
      // reset counter
      mapresettime = MAP_RESET_TIME;
    
     // htmap_max -> htmap_avg calucation
     for (uint8_t hum=0; hum<HT_MAP_COUNT_HUM; hum++)
     {
       for (uint8_t temp=0; temp<HT_MAP_COUNT_TEMP; temp++)
       {
         if (htmap_max[hum][temp]<65535)
         {
           htmap_avg[hum][temp]=(uint16_t)(((unsigned long)htmap_avg[hum][temp]*(MAP_RESET_AVG_COUNT-1)+(unsigned long)htmap_max[hum][temp])/MAP_RESET_AVG_COUNT);
           htmap_max[hum][temp]=65535;
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
        return 65535;
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
  
  // Write htmap
  lval = tmp;
  sethtmap(temp,humidity,tmp);
  
  // Rule check to avoid jumps in table, check actual cell against neighbours
  int maxval = htmap_avg[hpos][tpos];
  if ( (hpos>0) && (tpos>0) &&  (hpos<HT_MAP_COUNT_HUM-1) && (tpos<HT_MAP_COUNT_TEMP-1))
  {
    // hpos-1 must be less
    if (htmap_avg[hpos-1][tpos]>maxval) htmap_avg[hpos-1][tpos]=maxval;
    // tpos-1 must be less
    if (htmap_avg[hpos][tpos-1]>maxval) htmap_avg[hpos][tpos-1]=maxval;
    // hpos-1,tpos-1 must be less
    if (htmap_avg[hpos-1][tpos-1]>maxval) htmap_avg[hpos-1][tpos-1]=maxval;    
    // hpos+1 must be greater
    if (htmap_avg[hpos+1][tpos]<maxval) htmap_avg[hpos][tpos-1]=maxval;
    // tpos +1 must be greater
    if (htmap_avg[hpos][tpos+1]<maxval) htmap_avg[hpos][tpos+1]=maxval;
    // hpos+1,tpos +1 must be greater
    if (htmap_avg[hpos+1][tpos+1]<maxval) htmap_avg[hpos+1][tpos+1]=maxval;
    // hpos-1,tpos +1 must be greater
    if (htmap_avg[hpos-1][tpos+1]<maxval) htmap_avg[hpos-1][tpos+1]=maxval;
    // hpos+1,tpos-1 must be less
    if (htmap_avg[hpos+1][tpos-1]>maxval) htmap_avg[hpos+1][tpos-1]=maxval;
  }

//  quality = ( (float)maxval/(float)lval)*100;
  float f1 = lval-maxval;
  if (f1<0) f1=0;
  float f2 = (maxval*SENSOR_VARIANCE)-maxval;
  if (f2<0) f2=1;
  quality = 100-((f1*100)/f2);
  
  ltmp = temp;
  lhum = humidity;
  
  // Store quality history
  quality_history[quality_history_pos]=quality;
  quality_history_pos++;
  if (quality_history_pos>=QUALITY_HISTORY) quality_history_pos=0;
  
  return lval;
}
