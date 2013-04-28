#include <inttypes.h>
#include "modbuscrc.h"
#include "gasSensor.h"

// Debugging
#define DEBUG  1
#define LED_PIN 13

#define P300_BAUD_RATE 4800
#define PC_BAUD_RATE 115200

// in mS 1000/4800 Baud/10 * 3,5
#define MODBUS_TIMEOUT  7
#define READ_TIMEOUT  20

// time of inactivity before reading sensors
#define IDLE_TIMEOUT 100

// minimum time between sensor readings (16 bit!)
#define SENSOR_TIMEOUT 3000

// number of milliseconds for setup routine
#define SETUP_TIMEOUT 5000

// timeout for software watchdog
#define WATCHDOG_TIMEOUT 2000

// define I2C address when HYT-221 is installed
#define SENSOR_HYT 1
#ifdef SENSOR_HYT
  static char hy_addresses[SENSOR_HYT]={0x28};
#endif

// define analog pins for GAS Sensors
#define SENSOR_GAS 1
#ifdef SENSOR_GAS
  static int gassensor_pins[SENSOR_GAS]={14};
  GasSensor* gas_sensors[SENSOR_GAS];
#endif


// Commands
#define COMMAND_DELIMITER " "

// Define classes for serial communication
#define SERIAL_PC   Serial
#define SERIAL_P300 Serial3  
#define SERIAL_RC   Serial1

#define SERIAL_BUFFER_SIZE  64

// Data
struct ProxyObj
{
   uint8_t  buffer_rpos;
   uint8_t  buffer_wpos;
   char     buffer[SERIAL_BUFFER_SIZE];
   uint8_t  age;
   bool     recieve_from_p300;
   #ifdef DEBUG
   char*  name;
   #endif
};


// Error Codes
enum errorcodes
{
  ERR_PARAMETER_VALUE  
};

