/*
 * P300 Controller
 *
 * Belegung UART3 = P300
 *          UART1 = Remote Control Controller
 *
 *
 *
 *
 */
#include "config.h"
#include <stdarg.h>
#include <Wire.h>
#include <EEPROM.h>
#include <IntervalTimer.h>
#include "bitlash.h"
#include "modbuscrc.h"

#ifdef DEBUG
  bool debug;
#endif
  bool setupfinished;
#ifdef EEPROM_CRASHDEDECTION
  bool crashdedection;
#endif

ProxyObj proxy_rc;
ProxyObj proxy_intern;

ProxyObj* proxy_source;

ModbusCRC crc;

// Idle Timer
IntervalTimer Timer_ms;
IntervalTimer Timer_serial;
IntervalTimer Timer_clock;

uint16_t idle_timer;
uint16_t sensor_timeout;
uint16_t watchdog_timer;
bool watchdogsource;
bool inCallbackSerial;
uint8_t recursion_counter;

// HYT-221 sensors
#if defined(SENSOR_HYT) && SENSOR_HYT >= 1
  double hy_temp[SENSOR_HYT];
  double hy_humidity[SENSOR_HYT];
#endif

// http://playground.arduino.cc/Main/Printf
void p(char *fmt, ... ){
        char tmp[512]; // resulting string limited to 512 chars
        va_list args;
        va_start (args, fmt );
        vsnprintf(tmp, sizeof(tmp), fmt, args);
        va_end (args);
        Serial.print(tmp);
}

void setup() {
  #ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
  #endif
    
  // Crash -> reboot to boot loader
  #ifdef EEPROM_CRASHDEDECTION
  if (EEPROM.read(EEPROM_CRASHDEDECTION)<255)
  {
    #ifdef LED_PIN
     blink(BLINK_SETUP_FAIL);
    #endif
    EEPROM.write(EEPROM_CRASHDEDECTION,255);
    reset_to_bootloader();
  }
  // Don't reboot after flashing new firmware
  if (EEPROM.read(EEPROM_NEWFIRMWARE)<255)
  {
    EEPROM.write(EEPROM_CRASHDEDECTION,0);
    crashdedection=true;
  }
    else
  {
    EEPROM.write(EEPROM_NEWFIRMWARE,0);
    crashdedection=false;
  }
  #endif

  setupfinished=false;

  #ifdef DEBUG
    // default debug state
    debug=false;
  #endif

  // Setup serial ports
  SERIAL_P300.begin(P300_BAUD_RATE);
  SERIAL_RC.begin(P300_BAUD_RATE);
  crc=ModbusCRC();

  // Reset Proxy Objects
  resetProxyObj(&proxy_rc);
  resetProxyObj(&proxy_intern);
  #ifdef DEBUG
    proxy_rc.name    ="RC ";
    proxy_intern.name="INT";
  #endif

  // Define proxy_source
  proxy_source=NULL;
  
  // Set Idle timer
  idle_timer=IDLE_TIMEOUT;
  
  // Set Sensor timer
  sensor_timeout=SENSOR_TIMEOUT;
  
  // Watchdog timer
  watchdog_timer=0;
  watchdogsource=false;
  
  // Recursion counter
  recursion_counter=0;

  // Initialize 1ms timer with watchdog
  Timer_ms.begin(timerCallbackMs, 1000);
  
  // Initialize serial timer
  inCallbackSerial=false;
  Timer_serial.begin(timerCallbackSerial, 10000000/P300_BAUD_RATE);
  
  // Clock timer
  Timer_clock.begin(timerCallbackClock, 10000000);

  // Initialize I2C Bus
  Wire.begin();
  
  // Initialize gas sensors
  #if defined(SENSOR_GAS) && SENSOR_GAS >= 1
    for (char i=0;i<SENSOR_GAS;i++)
    {
       gas_sensors[i]=new GasSensor(i, gassensor_pins[i]);
    }
  #endif
  
  // initialize bitlash and set primary serial port baud
  // print startup banner and run the startup macro
  initBitlash(PC_BAUD_RATE);
  // Bitlash function
  addBitlashFunction("p300help", (bitlash_function) cmd_p300help);
  addBitlashFunction("sensor", (bitlash_function) cmd_sensor);
  addBitlashFunction("clock", (bitlash_function) cmd_clock);
  addBitlashFunction("modbus", (bitlash_function) cmd_modbus);
  #ifdef DEBUG
    addBitlashFunction("debug", (bitlash_function) cmd_debug);
  #endif
  addBitlashFunction("flash", (bitlash_function) reset_to_bootloader);

  
  #ifdef LED_PIN
    digitalWrite(LED_PIN, LOW);
  #endif
  
  setupfinished=true;
}

// Reset function
numvar reset_to_bootloader()
{
  #ifdef LED_PIN
   digitalWrite(LED_PIN, LOW);
  #endif

  cli();
  delay(100);
  _reboot_Teensyduino_();
}


// reboot function
void reboot()
{
  #ifndef SCB_AIRCR_SYSRESETREQ_MASK
    #define SCB_AIRCR_SYSRESETREQ_MASK ((unsigned int) 0x00000004)
  #endif

  #ifdef LED_PIN
   digitalWrite(LED_PIN, LOW);
  #endif

  cli();
  delay(100);
  SCB_AIRCR = 0x05FA0000 | SCB_AIRCR_SYSRESETREQ_MASK;
  while(1);
}

// bitlash p300help
numvar cmd_p300help(void)
{
        p(
          "P300 Controller Build=\"" __DATE__ " " __TIME__ "\"\r\n"
          "USE IT AT YOUR OWN RISK!!\r\n"
          "Commands:\r\n"
          "p300help\r\n"
          "flash\t\tReboot to loader\r\n"
          "boot\t\tReboot to application\r\n"
         );
        #ifdef DEBUG
          p(
            "debug(0|1)\tdisable|enable debug messages\r\n"
            "debug(10)\tDump internal variables\r\n"
          ); 
        #endif
         p("sensor(TYPE,NUM[,MULTIPLICATOR])\r\n\tRead value of sensor TYPE,NUM\r\n\tTYPE=1\tInternal temperature sensor\r\n");
        #if defined(SENSOR_HYT) && SENSOR_HYT >= 1
         p("\tTYPE=2\tExternal temperature sensor\r\n\tTYPE=3\tEnternal humidity sensor\r\n");
        #endif
        #if defined(SENSOR_GAS) && SENSOR_GAS >= 1
         p("\tTYPE=4\tEnternal air quality sensor absolute value\r\n\tTYPE=5\tEnternal air quality sensor relative value\r\n\tTYPE=6\tEnternal air quality sensor 1 minute relative delta\r\n");
        #endif
        p("clock(VAL)\tread clock 0=second,1=minute,2=hour,3=weekday -> 0=Sun-6=Sat\r\nmodbus(addr[,value])\tread or write word from/to P30 register\r\n");
  return 0;
}

// bitlash sensor function
numvar cmd_sensor(void)
{
  numvar ret=-100;
  if (getarg(0)>=2)
  {
    float fret = -200;
    char n=getarg(2)-1;
    switch (getarg(1))
    {
        // Internal P300 sensor
        case 1:
          break;
        #if defined(SENSOR_HYT) && SENSOR_HYT >= 1
        // External HYT Sensor
        case 2:
          if ( (n>=0) && (n<SENSOR_HYT)) fret=hy_temp[n];
          break;
        case 3:
          if ( (n>=0) && (n<SENSOR_HYT)) fret=hy_humidity[n];
          break;
        #endif      
        #if defined(SENSOR_GAS) && SENSOR_GAS >= 1
        // Gas sensor
        case 4:
          if ( (n>=0) && (n<SENSOR_GAS)) fret=gas_sensors[n]->getValue();
          break;
        case 5:
          if ( (n>=0) && (n<SENSOR_GAS)) fret=gas_sensors[n]->getQuality();
          break;
        case 6:
          fret=0;
          break;
        #endif      
    }
    // multiplication
    if (getarg(0)>2)
    {
      ret=(numvar)(fret*getarg(3));
    }
      else
    {
       ret=(numvar)fret;
    }
    
  }
  return ret;
}

// bitlash sensor function
numvar cmd_clock(void)
{
  if (getarg(0)==1)
  {
  }
  return -1;
}

// bitlash modbus function
numvar cmd_modbus(void)
{
  // endless loop protection
  if (recursion_counter>3) return -2;
  recursion_counter++;
  
  char numarg = getarg(0);
  numvar ret = -1;
  char waitforbytes = 7; // read answer
  
  // One or two arguments
  if ( (numarg>0) && (numarg<3))
  {
    // Ser proxyObj to a known state
    resetProxyObj(&proxy_intern);
   
  
    // Reset CRC
    crc.clearCRC();
      
    // Write Modbus Address
    addToProxyObj(&proxy_intern, 0x01, false);
    crc.add_byte(0x01);
  
    if (numarg==1)
    {
      // Function Code: 0x03 Read Holding Registers
      addToProxyObj(&proxy_intern, 0x03, false);
      crc.add_byte(0x03);
    }
      else
    {
      // Function Code: 0x10 Preset Multiple Registers
      addToProxyObj(&proxy_intern, 0x10, false);
      crc.add_byte(0x10);  
    }
    
    // Write adress
    addWordToProxyObj(&proxy_intern, (uint16_t)getarg(1), &crc);

    // Number of registers to read/write
    addWordToProxyObj(&proxy_intern, 0x0001, &crc);
    
    // Write
    if (numarg==2)
    {
      // Two bytes following
      addToProxyObj(&proxy_intern, 0x02, false);
      crc.add_byte(0x02);
      
      addWordToProxyObj(&proxy_intern, (uint16_t)getarg(2), &crc);
      
      waitforbytes=8; // write answer
    }
    
    // CRC senden
    addWordToProxyObj(&proxy_intern, crc.getCRC(), NULL);
    
    // wait send + recieve time
    delay( ((proxy_intern.buffer_wpos+waitforbytes)*1000)/480 );
    
    // Wait for end of send packet 
    #ifdef DEBUG
      if (debug) p("D Wait for end of transmission\r\n");
    #endif

    while ( proxy_intern.transmission )
    {
       delay(MODBUS_TIMEOUT);
    }
    
    // set rpos to wpos;
    proxy_intern.buffer_rpos=proxy_intern.buffer_wpos;

    // Reset wpos if nothing recieved
    if (proxy_intern.recieve_from_p300==false) proxy_intern.buffer_wpos=0;
    
    #ifdef DEBUG
      if (debug) p("D Stop wait for answer. %d bytes in buffer\r\n",proxy_intern.buffer_wpos);
    #endif
    
    if (proxy_intern.buffer_wpos>0)
    {
      // Check answer
      crc.clearCRC();
      uint8_t i =0;
      for (;i<proxy_intern.buffer_wpos-2;i++) crc.add_byte(proxy_intern.buffer[i]);
      if (crc.compareCRC(proxy_intern.buffer[i++],proxy_intern.buffer[i++]))
      {
        #ifdef DEBUG
          if (debug) p("D CRC OK\r\n");
        #endif
      
        if (numarg==2)
        {
           // write
          ret = 1;
        }
          else
        {
          // read
          ret = ((proxy_intern.buffer[3]*256) + proxy_intern.buffer[4]);
        }
      }
       else
      {   
        #ifdef DEBUG
          if (debug) p("D CRC error\r\n");
        #endif

        // Wair for timeout for recursion call
        while (proxy_intern.age<READ_TIMEOUT)
        {
          // Sleep until next interrupt
          asm volatile("wfi\r\n"::);
        }
      
        // recursive call
        ret = cmd_modbus();
      }
    }
  }
  
  // Wrong arguments
  recursion_counter--;
  return ret;
}

// bitlash debug function
#ifdef DEBUG
numvar cmd_debug(void)
{
  numvar ret=(numvar)debug;
  if (getarg(0)==1)
  {
    numvar arg = getarg(1);
    switch (arg)
    {
      case 0:
        debug=false;
        break;
      case 1:
        debug=true;
        break;
      case 10:
          p(
           "uptime=%ds\r\n"
           "proxy_source=%s\r\n"
           "proxy_rc.wpos rpos age=%d %d %d\r\n"
           "proxy_intern.wpos rpos age=%d %d %d\r\n"
           "rc.available=%d\r\n"
           "p300.available=%d\r\n"
           "idle_timer=%d\r\n"
           "sensor_timeout=%d\r\n"
           "watchdog_timer=%d\r\n"
           ,millis()/1000,
           (proxy_source==NULL)?"NULL":proxy_source->name,
           proxy_rc.buffer_wpos,proxy_rc.buffer_rpos,proxy_rc.age,
           proxy_intern.buffer_wpos,proxy_intern.buffer_rpos,proxy_intern.age,
           SERIAL_RC.available(),
           SERIAL_P300.available(),
           idle_timer,
           sensor_timeout,
           watchdog_timer
         );
        break;
    }
  }
  return ret;
}
#endif

#ifdef LED_PIN
void blink(char n)
{
  for (char i=0;i<30;i++)
  {
   digitalWrite(LED_PIN, HIGH);
   delay(40);
   digitalWrite(LED_PIN, LOW);
   delay(40);
  }
  delay(500);
  for (char i=0;i<n;i++)
  {
   digitalWrite(LED_PIN, HIGH);
   delay(500);
   digitalWrite(LED_PIN, LOW);
   delay(500);
  }
}
#endif


void loop()
{
   #ifdef LED_PIN
    digitalWrite(LED_PIN, HIGH);
   #endif

   // Wechselwatchdog
   if (watchdogsource)
   {
       watchdog_timer=0;
       watchdogsource=false;
   }

   runBitlash();
  
  // Read Sensors
  if ( (idle_timer>IDLE_TIMEOUT) && ((sensor_timeout>=SENSOR_TIMEOUT)))
  {
    #ifdef DEBUG
      if (debug) p("D Start reading sensors %d\r\n",millis());
    #endif
    
    #if defined(SENSOR_HYT) && SENSOR_HYT >= 1
    for (char num_sensor=0;num_sensor<SENSOR_HYT;num_sensor++)
    {
      readHYT(hy_addresses[num_sensor], &hy_temp[num_sensor], &hy_humidity[num_sensor]);
    }
    #endif
    
    #if defined(SENSOR_GAS) && SENSOR_GAS >= 1
    for (char i=0;i<SENSOR_GAS;i++)
    {
      uint16_t gasval = gas_sensors[i]->loopAction((char)hy_temp[i],(char)hy_humidity[i]);
      #ifdef DEBUG
       if (debug) p("D gas sensor %d=%d\r\n",i,gasval);
      #endif
    }
    #endif

    #ifdef DEBUG
      if (debug) p("D End reading sensors %d\r\n",millis());
    #endif
    sensor_timeout=0;
  }

  // Crash detection, release EEPROM Value after 30 Seconds uptime
  #ifdef EEPROM_CRASHDEDECTION
  if ( (crashdedection==true) && (millis()>CRASH_DETECTION_TIMEOUT))
  {
    if (EEPROM.read(EEPROM_CRASHDEDECTION)<255) EEPROM.write(EEPROM_CRASHDEDECTION,255);
    crashdedection=false;
  }
  #endif  
  
   #ifdef LED_PIN
    digitalWrite(LED_PIN, LOW);
   #endif

  // Sleep until next interrupt
  asm volatile("wfi\r\n"::);
}

#if defined(SENSOR_HYT) && SENSOR_HYT >= 1
void readHYT(char address, double *temp, double *humidity)
{
    // Read 4 bytes from HYT sensor
    uint8_t buffer_pos;
    uint8_t buffer[4];
    
    // Send Measurement Request
    Wire.beginTransmission(address);
    if (Wire.write(0)!=1)
    {
      #ifdef DEBUG
        if(debug) p("D Error writing to HYT sensor %x\r\n",address);
      #endif
      return;
    }
    Wire.endTransmission();
    
    // Read data
    Wire.requestFrom(address, 4);
    for (buffer_pos=0;buffer_pos<4;buffer_pos++)
    {
      if (Wire.available()) buffer[buffer_pos]=Wire.read();
        else break;
    }
    if (buffer_pos==4)
    {
      // Check for answer
      if ( (buffer[0]+buffer[1]+buffer[2]+buffer[3])>0)
      {
        // Calculate values
        *temp = 165.0/pow(2,14)*(buffer[2] << 6 | (buffer[3] & 0x3F))-40;
        *humidity = 100/pow(2,14)*((buffer[0]<<8 | buffer[1]) & 0X3FFF);
      }
        else
      {
        // Set dummy values
        *temp = 20;
        *humidity = 50;
      }
      #ifdef DEBUG
        if(debug) p("D Read from HYT sensor %x temp=%f humidity=%f\r\n",address,*temp,*humidity);
      #endif
    }
    #ifdef DEBUG
      else if(debug)
    {
      p("D Error read from HYT sensor %x",address);
    }
    #endif
}
#endif

// Initialize ProxyObj with defined values
void resetProxyObj(ProxyObj *obj)
{
  // Initialize Buffer
  obj->buffer_rpos=0;
  obj->buffer_wpos=0;
  obj->age=0;
  obj->recieve_from_p300=false;
  obj->transmission=false;
  #ifdef DEBUG
    if (debug)
    {
      p("D Reset %s\r\n",obj->name);
    }
  #endif
}

// Add Data to ProxyObj
void addToProxyObj(ProxyObj *obj, uint8_t data, bool recieve_from_p300)
{
  // Reset idle timer
  idle_timer=0;
  
  // wrong call of addToProxyObj
  if (obj==NULL)
  {
  #ifdef DEBUG
    if (debug)
    {
      p("D NULL %c BUFFER %X (NULL)\r\n",recieve_from_p300 ?'<':'>',data);
    }
  #endif
    return;
  }
  
  // Reset ProxyObj if direction changes or timeout is reaced
  if ( (recieve_from_p300!=obj->recieve_from_p300) || (obj->age>READ_TIMEOUT))
  {
     resetProxyObj(obj);
     obj->recieve_from_p300=recieve_from_p300;     
  }
    else
  {
     // reset transmission and age
     obj->transmission = true;
     obj->age = 0;
  }

  #ifdef DEBUG
    if (debug)
    {
      p("D %s %c BUFFER %X (%d)\r\n",obj->name,obj->recieve_from_p300 ?'<':'>',data, obj->buffer_wpos);
    }
  #endif
  obj->buffer[obj->buffer_wpos]=data;
  if (obj->buffer_wpos < sizeof(obj->buffer)) obj->buffer_wpos++;
}

void addWordToProxyObj(ProxyObj *obj, uint16_t data,ModbusCRC *crc)
{
  uint8_t h = data >> 8;
  uint8_t l = data & 0xff;
  addToProxyObj(obj, h, false);
  addToProxyObj(obj, l, false);
  
  if (crc!=NULL)
  {
    crc->add_byte(h);
    crc->add_byte(l);
  }
}


// 1ms second timer
void timerCallbackMs() {
  // Increment age of data
  if (proxy_rc.buffer_wpos>0)
  {
      proxy_rc.age++;
      if (proxy_rc.age>MODBUS_TIMEOUT) proxy_rc.transmission=false;
  }
  if (proxy_intern.buffer_wpos>0)
  {
     proxy_intern.age++;
      if (proxy_intern.age>MODBUS_TIMEOUT) proxy_intern.transmission=false;
  }
  idle_timer++;
  if (sensor_timeout<=SENSOR_TIMEOUT) sensor_timeout++;
  
  // Software watchdog
  watchdog_timer++;  
  // reboot into program mode
  if ((!setupfinished) && (watchdog_timer>SETUP_TIMEOUT))
  {
    #ifdef LED_PIN
    cli();
    blink(BLINK_SETUP_FAIL_TIMEOUT);
    #endif
    reset_to_bootloader();
  }
  
  // Software watchdog timeout
  if ((setupfinished) && (watchdog_timer>WATCHDOG_TIMEOUT))
  {
    #ifdef LED_PIN
    cli();
    blink(BLINK_SOFTWARE_WATCHDOG);
    #endif
    // Reset
    reboot();
  }
}

// Serial Timer
void timerCallbackSerial()
{
   if (inCallbackSerial) return;
   inCallbackSerial=true;
   
   // Wechselwatchdog
   if (!watchdogsource)
   {
       watchdog_timer=0;
       watchdogsource=true;
   }

   
  /*********************************************************
   * Serial Proxy handling
   *********************************************************/

  // free proxy_source if data to old
  if ( (proxy_source!=NULL) && (proxy_source->age>READ_TIMEOUT))
  {
    #ifdef DEBUG
       if (debug) p("D proxy_source=NULL timeout=%d\r\n",proxy_source->age);
    #endif
    proxy_source=NULL;
  }
    
  // Assign proxy_source
  if (proxy_source==NULL)
  {
    // Data in buffer
    if (proxy_rc.buffer_wpos>proxy_rc.buffer_rpos)
    {
       proxy_source=&proxy_rc;
    }
    // Data in internal buffer
    if (proxy_intern.buffer_wpos>proxy_intern.buffer_rpos)
    {
       proxy_source=&proxy_intern;
    }
   
   // Flush old data
   if (proxy_source!=NULL)
   {
     SERIAL_P300.flush(); // send outstanding data
     SERIAL_P300.clear(); // clear buffer
     #ifdef DEBUG
         if (debug) p("D proxy_source=%s\r\n",proxy_source->name);
     #endif
     proxy_source->age=0;
   }
  }

  if (proxy_source!=NULL)
  {
   // Read data from P300
   while (SERIAL_P300.available())
   {
     addToProxyObj(proxy_source, SERIAL_P300.read(), true);
   }
   
   // Send data to P300
   if ( (proxy_source->buffer_wpos>proxy_source->buffer_rpos) && (proxy_source->recieve_from_p300==false))
   {
    char c = proxy_source->buffer[proxy_source->buffer_rpos];
    SERIAL_P300.write(c);
    #ifdef DEBUG
      if (debug) p("D %s > P300 %X (%d)\r\n",proxy_source->name,c,proxy_source->buffer_rpos);
    #endif
    proxy_source->buffer_rpos++;
    proxy_source->age=0;
   }
  }
  
  // Read serial Ports
  while (SERIAL_RC.available())
  {
    addToProxyObj(&proxy_rc, SERIAL_RC.read(), false);
  }
  
  // Write to serial Ports
  if ( (proxy_rc.buffer_wpos>proxy_rc.buffer_rpos) && (proxy_rc.recieve_from_p300))
  {
     char c = proxy_rc.buffer[proxy_rc.buffer_rpos];
     SERIAL_RC.write(c);
     #ifdef DEBUG
       if (debug) p("D BUFFER > RC  %X (%d)\r\n", c, proxy_rc.buffer_rpos);
     #endif
     proxy_rc.buffer_rpos++;
     proxy_rc.age=0;
  }
  
  inCallbackSerial=false;
}

void timerCallbackClock()
{
}
