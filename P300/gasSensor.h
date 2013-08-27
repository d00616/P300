/*
 * Gas Sensor Round Robin Library
 *
 */
#ifndef gassensor_h
#define gassensor_h

// Num of reads for analogReadAverage
#define NUM_ANALOG_READS 10

// divder for temperature and humidity (htmap)
#define HT_MAP_DIV_TEMP 3
#define HT_MAP_MIN_TEMP 0
#define HT_MAP_MAX_TEMP 30
#define HT_MAP_COUNT_TEMP (uint8_t)((HT_MAP_MAX_TEMP-HT_MAP_MIN_TEMP)/HT_MAP_DIV_TEMP)+1
#define HT_MAP_DIV_HUM 5
#define HT_MAP_MIN_HUM 10
#define HT_MAP_MAX_HUM 100
#define HT_MAP_COUNT_HUM (uint8_t)((HT_MAP_MAX_HUM-HT_MAP_MIN_HUM)/HT_MAP_DIV_HUM)+1

#define MAX_QUALITY_JUMP 5

// Time in ms to first messurement (15 min in ms)
#define SENSOR_GAS_PREHEAT_TIME  900000
// s
#define MAP_RESET_TIME 86400

#include <inttypes.h>

 
class GasSensor
{
  protected:
    int  pin; // Pin Number
    bool pinhigh; // Pin voltage > 1.2V
    int lval; // Last value
    char quality, ltmp, lhum;
    uint32_t lmillis;
    
    
    // inital Humidity/Temperature map
    // TGS2600 datasheet
#ifndef ARDUINO
  public:
     uint32_t millis_val;
     uint16_t analog_val;
     
     uint32_t millis();
     uint16_t analogRead(int);
#endif
    uint16_t htmap[HT_MAP_COUNT_HUM][HT_MAP_COUNT_TEMP];
    uint16_t htmap_time[HT_MAP_COUNT_HUM][HT_MAP_COUNT_TEMP];
    int32_t mapresettime;

  public:
   // initialize gas sensor with pin (int)
   GasSensor(char, int);
   // read actual value
   uint16_t getValue();
   // read actual value
   int8_t getQuality();
   // loop action
   uint16_t loopAction(char, char);
   
  private:
   void sethtmap(char,char, uint16_t);
    
   
};
#endif
