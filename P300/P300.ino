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


// https://github.com/loglow/PITimer.git
#include "PITimer.h"

#ifdef DEBUG
  bool debug;
#endif
  bool setupfinished;
#ifdef EEPROM_CRASHDEDECTION
  bool crashdedection;
#endif


ProxyObj proxy_rc;
ProxyObj proxy_pc;
ProxyObj proxy_intern;

ProxyObj* proxy_source;

// End of line
bool  PcEol;

// Idle Timer
uint16_t idle_timer;
uint16_t sensor_timeout;
uint16_t watchdog_timer;

// HYT-221 sensors
#if defined(SENSOR_HYT) && SENSOR_HYT >= 1
  double hy_temp[SENSOR_HYT];
  double hy_humidity[SENSOR_HYT];
#endif

// http://playground.arduino.cc/Main/Printf
void p(char *fmt, ... ){
        char tmp[256]; // resulting string limited to 128 chars
        va_list args;
        va_start (args, fmt );
        vsnprintf(tmp, 128, fmt, args);
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
  EEPROM.write(EEPROM_CRASHDEDECTION,0);
  crashdedection=true;
  #endif

  setupfinished=false;

  #ifdef DEBUG
    // default debug state
    debug=false;
  #endif

  // Setup serial ports
  SERIAL_PC.begin(PC_BAUD_RATE);
  SERIAL_P300.begin(P300_BAUD_RATE);
  SERIAL_RC.begin(P300_BAUD_RATE);

  // Reset Proxy Objects
  resetProxyObj(&proxy_rc);
  resetProxyObj(&proxy_pc);
  resetProxyObj(&proxy_intern);
  #ifdef DEBUG
    proxy_rc.name    ="RC ";
    proxy_pc.name    ="PC ";
    proxy_intern.name="INT";
  #endif

  // Define proxy_source
  proxy_source=NULL;
  
  // Reset PcEol;
  PcEol = false;
  
  // Set Idle timer
  idle_timer=IDLE_TIMEOUT;
  
  // Set Sensor timer
  sensor_timeout=SENSOR_TIMEOUT;
  
  // Watchdog timer
  watchdog_timer=0;

  // Initialize 1ms timer with watchdog
  PITimer2.frequency(1000);
  PITimer2.start(timerCallbackMs);  

  // Initialize I2C Bus
  Wire.begin();
  
  // Initialize gas sensors
  #if defined(SENSOR_GAS) && SENSOR_GAS >= 1
    for (char i=0;i<SENSOR_GAS;i++)
    {
       gas_sensors[i]=new GasSensor(gassensor_pins[i]);
    }
  #endif
  
  #ifdef LED_PIN
    digitalWrite(LED_PIN, LOW);
  #endif
  
  setupfinished=true;
}

void cmd_ok()
{
  p("OK\r\n");
}

void cmd_error(int n)
{
  p("ERR %d\r\n",n);
}

// Reset function
void reset_to_bootloader()
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
   watchdog_timer=0;
   
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
    // Modbus data (slave address 01) in buffer
    if ( (proxy_pc.buffer_wpos>proxy_pc.buffer_rpos) && (proxy_pc.buffer[0]==0x01))
    {
       proxy_source=&proxy_pc;
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
   }
  }

  if (proxy_source!=NULL)
  {
   // Read data from P300
   if (SERIAL_P300.available())
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
   }
  }
    
  
  // Read serial Ports
  if (SERIAL_RC.available())
  {
    addToProxyObj(&proxy_rc, SERIAL_RC.read(), false);    
  }
  if (SERIAL_PC.available())
  {
    char c = SERIAL_PC.read();
    // End of line
    if ( (c=='\n') || (c=='\r')) PcEol=true;
      else PcEol=false;
    addToProxyObj(&proxy_pc, c, false);    
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
  }

  if ( (proxy_pc.buffer_wpos>proxy_pc.buffer_rpos)  && (proxy_pc.recieve_from_p300))
  {
     char c = proxy_pc.buffer[proxy_pc.buffer_rpos];
     SERIAL_PC.write(c);
     #ifdef DEBUG
       if (debug) p("D BUFFER > RC  %X (%d)\r\n", c, proxy_pc.buffer_rpos);
     #endif
     proxy_pc.buffer_rpos++;
  }
  
  /*********************************************************
   * Handle commands
   *********************************************************/
  if ( (PcEol) && (proxy_pc.buffer[0]!=0x01)) // Command send
  {
    // add 0x00 to string end
    proxy_pc.buffer[proxy_pc.buffer_wpos]=0;
    
    // Pointer to arguments
    char delim[6]=" =\r\n\0";
    char *last=NULL;
    char *cmd=strtok_r((char*)proxy_pc.buffer,delim, &last);
    char *arg1=strtok_r(NULL,delim, &last);
    char *arg2=strtok_r(NULL,delim, &last);

    #ifdef DEBUG
    if (debug)
    {
      p("D cmd=%s\targ1=%s\targ2=%s\tstrncmp(DEBUG)=%d\r\n",cmd,arg1,arg2,strncmp((char*)proxy_pc.buffer,"DEBUG",5));
    }
    #endif

    // HELP
    if (proxy_pc.buffer[0]=='?')
    {
        p(
          "P300 Controller Build=\"" __DATE__ " " __TIME__ "\"\r\n"
          "Commands:\r\n"
          "?\t\tHelp\r\n"
          "FLASH\t\tReboot to loader\r\n"
         );
        #ifdef DEBUG
          p("DEBUG\t0|1\tdisable|enable debug messages\r\n"); 
        #endif
        #if defined(SENSOR_HYT) && SENSOR_HYT >= 1
         p("GETHT\tNUM\tRead external HYT-221 sensor NUM\r\n");
        #endif
        #if defined(SENSOR_GAS) && SENSOR_GAS >= 1
         p("GETGAS\tNUM\tRead external gas sensor NUM\r\n");
        #endif
    }
    
    // DEBUG
    #ifdef DEBUG
     if (strncmp(cmd,"DEBUG",5) == 0)
     {
       if (arg1[0]=='0')
       {
         debug=false;
         cmd_ok();
       }
         else
       if (arg1[0]=='1')
       {
         debug=true;
         cmd_ok();
       }
         else
       {
        cmd_error(ERR_PARAMETER_VALUE);
       }
     }
    #endif

    // Boot loader
    if (strncmp(cmd,"FLASH",5) == 0)
    {
      cmd_ok();
      reset_to_bootloader();
    }

    #if defined(SENSOR_HYT) && SENSOR_HYT >= 1
     if (strncmp(cmd,"GETHT",5) == 0)
     {
       char n=atoi(arg1)-1;
       if ( (n>=0) && (n<=SENSOR_HYT))
       {
         p("HT%d temperature=%f humidity=%f\r\n",n+1,hy_temp[n],hy_humidity[n]);
         cmd_ok();
       }
         else
       {
        cmd_error(ERR_PARAMETER_VALUE);
       }
     }
    #endif

    #if defined(SENSOR_GAS) && SENSOR_GAS >= 1
     if (strncmp(cmd,"GETGAS",6) == 0)
     {
       char n=atoi(arg1)-1;
       if ( (n>=0) && (n<=SENSOR_GAS))
       {
         p("GAS%d value=%d quality=%d\r\n",n+1,gas_sensors[n]->getValue(),gas_sensors[n]->getQuality());
         cmd_ok();
       }
         else
       {
        cmd_error(ERR_PARAMETER_VALUE);
       }
     }
    #endif


    resetProxyObj(&proxy_pc);
    PcEol=false;
  }
  
  
  // Read Sensors
  if ( (idle_timer>IDLE_TIMEOUT) && ((sensor_timeout>=SENSOR_TIMEOUT)))
  {
    #ifdef DEBUG
      if (debug) p("D Start reading sensors %d\r\n",millis());
    #endif
    
    #if defined(SENSOR_HYT) && SENSOR_HYT >= 1
    for (char num_sensor=0;num_sensor<SENSOR_HYT;num_sensor++)
    {
//      readHYT(hy_addresses[num_sensor], &hy_temp[num_sensor], &hy_humidity[num_sensor]);
    }
    #endif
    
    #if defined(SENSOR_GAS) && SENSOR_GAS >= 1
    for (char i=0;i<SENSOR_GAS;i++)
    {
      #ifdef DEBUG
       if (debug) p("D gas sensor %d=%d\r\n",i,gas_sensors[i]->loopAction());
         else gas_sensors[i]->loopAction();
      #else
       gas_sensors[i]->loopAction();
      #endif
    }
    #endif

    #ifdef DEBUG
      if (debug) p("D End reading sensors %d\r\n",millis());
    #endif
    sensor_timeout=0;
  }
  
  
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
    Wire.send(0);
    Wire.receive();
    Wire.endTransmission();
    
    // Read data
    Wire.requestFrom(address, 4);
    for (buffer_pos=0;buffer_pos<4;buffer_pos++)
    {
      if (Wire.available()) buffer[buffer_pos]=Wire.read();
        else buffer_pos=255;
    }
    if (buffer_pos<255)
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
//     if (proxy_source==obj) proxy_source=NULL;
  }

  #ifdef DEBUG
    if (debug)
    {
      p("D %s %c BUFFER %X (%d)\r\n",obj->name,obj->recieve_from_p300 ?'<':'>',data, obj->buffer_wpos);
    }
  #endif
  obj->buffer[obj->buffer_wpos]=data;
  if (obj->buffer_wpos < sizeof(obj->buffer)) obj->buffer_wpos++;
  obj->age=0;

  // Reset idle timer
  idle_timer=0;
}

// 1ms second timer
void timerCallbackMs() {
  // Increment age of data
  if (proxy_rc.buffer_wpos>0) proxy_rc.age++;
  if ((proxy_pc.buffer_wpos>0) && (proxy_pc.buffer[0]==0x01)) proxy_pc.age++;
  if (proxy_intern.buffer_wpos>0) proxy_intern.age++;
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

