#if defined(__AVR__)
#include <avr/pgmspace.h>
#include <util/delay.h>
#else
#include "pgmspace.h"
#endif
#include <stdlib.h>

#include "Chirp_Sensor_U.h"

/*========================================================================*/
/*                          PRIVATE FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    @brief  Writes a register and an 8 bit value over I2C
*/
/**************************************************************************/
void Chirp_Sensor_Unified::write8 (uint8_t reg, uint32_t value)
{
  Wire.beginTransmission(_addr);
  #if ARDUINO >= 100
  Wire.write(reg);
  Wire.write(value & 0xFF);
  #else
  Wire.send(reg);
  Wire.send(value & 0xFF);
  #endif
  Wire.endTransmission();
}

void Chirp_Sensor_Unified::write8 (uint8_t reg)
{
  Wire.beginTransmission(_addr);
  #if ARDUINO >= 100
  Wire.write(reg);
  #else
  Wire.send(reg);
  #endif
  Wire.endTransmission();
}


/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
uint8_t Chirp_Sensor_Unified::read8(uint8_t reg)
{
  Wire.beginTransmission(_addr);
  #if ARDUINO >= 100
  Wire.write(reg);
  #else
  Wire.send(reg);
  #endif
  Wire.endTransmission();

  Wire.requestFrom(_addr, 1);
  #if ARDUINO >= 100
  return Wire.read();
  #else
  return Wire.receive();
  #endif
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/
uint16_t Chirp_Sensor_Unified::read16(uint8_t reg)
{
  uint16_t x; uint16_t t;

  Wire.beginTransmission(_addr);
  #if ARDUINO >= 100
  Wire.write(reg);
  #else
  Wire.send(reg);
  #endif
  Wire.endTransmission();

  Wire.requestFrom(_addr, 2);
  #if ARDUINO >= 100
  t = Wire.read();
  x = Wire.read();
  #else
  t = Wire.receive();
  x = Wire.receive();
  #endif
  
//  x <<= 8;
//  x |= t;
//  return x;

//  unsigned int t = Wire.read() << 8;
//  t = t | Wire.read();
//  return t;
  t <<= 8;
  t |= x;
  return t;
}

/**************************************************************************/
/*!
    Enables the device
*/
/**************************************************************************/
void Chirp_Sensor_Unified::enable(void)
{
  /* Enable the device by setting the control bit to 0x03 */
//  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWERON);
}

/**************************************************************************/
/*!
    Disables the device (putting it in lower power sleep mode)
*/
/**************************************************************************/
void Chirp_Sensor_Unified::disable(void)
{
  /* Turn the device off to save power */
//  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWEROFF);
}

/**************************************************************************/
/*!
    Private function to read luminosity on both channels
*/
/**************************************************************************/
//void Chirp_Sensor_Unified::getData (uint16_t *broadband, uint16_t *ir)
//{
//  /* Enable the device by setting the control bit to 0x03 */
//  enable();
////
////  /* Wait x ms for ADC to complete */
////  switch (_tsl2561IntegrationTime)
////  {
////    case TSL2561_INTEGRATIONTIME_13MS:
////      delay(TSL2561_DELAY_INTTIME_13MS);  // KTOWN: Was 14ms
////      break;
////    case TSL2561_INTEGRATIONTIME_101MS:
////      delay(TSL2561_DELAY_INTTIME_101MS); // KTOWN: Was 102ms
////      break;
////    default:
////      delay(TSL2561_DELAY_INTTIME_402MS); // KTOWN: Was 403ms
////      break;
////  }
//
//  /* Reads a two byte value from channel 0 (visible + infrared) */
//  *broadband = read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW);
//
//  /* Reads a two byte value from channel 1 (infrared) */
//  *ir = read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW);
//
//  /* Turn the device off to save power */
//  disable();
//}

/*========================================================================*/
/*                            CONSTRUCTORS                                */
/*========================================================================*/

/**************************************************************************/
/*!
    Constructor
*/
/**************************************************************************/
Chirp_Sensor_Unified::Chirp_Sensor_Unified(uint8_t addr, int32_t sensorID) 
{
  _addr = addr;
  _chirpInitialised = false;
  _chirpSensorID = sensorID;
}

/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    Initializes I2C and configures the sensor (call this function before
    doing anything else)
*/
/**************************************************************************/
boolean Chirp_Sensor_Unified::begin(void) 
{
  Wire.begin();

  /* Make sure we're actually connected */
//  uint8_t x = read8(CHIRP_GET_VERSION);
//  if (!(x & 0x23)) // FIRMWARE_VERSION 0x23 //2.3
//  {
//    return false;
//  }

  write8(CHIRP_RESET);
  _chirpInitialised = true;

  /* Note: by default, the device is in power down mode on bootup */
  disable();

  return true;
}

void Chirp_Sensor_Unified::getCapacitance(uint16_t *capacitanceSensor) {
  *capacitanceSensor = read16(CHIRP_GET_CAPACITANCE);
//  Serial.print("Chirp Capacitance:"); Serial.println(*capacitanceSensor);
}

float Chirp_Sensor_Unified::calculateRelativeHumidity(uint16_t capacitance) {
  // Water = ~800
  // Air = ~320
  return (float)(map(capacitance, 300, 850, 0, 10000)) / 100;
}

void Chirp_Sensor_Unified::getTemperature(float *temperature) {
  uint16_t sensor = read16(CHIRP_GET_TEMPERATURE);
//  Serial.print("Chirp Temp:"); Serial.println(sensor);
  *temperature = calculateCelsius(sensor);
}

float Chirp_Sensor_Unified::calculateCelsius(uint16_t temperatureSensor) {
// 231 == 21.00*C * 11.
  return (float)temperatureSensor / 11;
//  return (float)(map(temperatureSensor, 0, 65535, 0, 10000)) / 100;
}

void Chirp_Sensor_Unified::getLight(float *light) {
  write8(CHIRP_MEASURE_LIGHT);
  uint16_t sensor = read16(CHIRP_GET_LIGHT);
//  Serial.print("Chirp Light:"); Serial.println(sensor);
  *light = calculateLux(sensor); // / 100; //map(sensor, 0, 65535, 0, 10000) / 100;
}

float Chirp_Sensor_Unified::calculateLux(uint16_t lightSensor) {
  // Input scale -- Dark : 65535 -> 0 : Bright
  // Output scale - Dark : 0 -> 300 : Bright     <-- Requires calibration.
  return (float)(map(lightSensor, 65535, 0, 0, 30000)) / 100;
  // https://en.wikipedia.org/wiki/Lux
  // 0.0001 lux  Moonless, overcast night sky (starlight)[3]
  // 50 Family living room lights (Australia, 1998)[7]
  // 100 Very dark overcast day[3]
  // 320–500 Office lighting[7][10][11][12]
  // 400 Sunrise or sunset on a clear day.
  // 10,000–25,000 Full daylight (not direct sun)[3]
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
    returns true if sensor reading is between 0 and 65535 lux
    returns false if sensor is saturated
*/
/**************************************************************************/
bool Chirp_Sensor_Unified::getEvent(sensors_event_t *event)
{
  uint16_t capacitance, temperature, light;
  
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));
  
  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _chirpSensorID;
  event->type      = SENSOR_TYPE_RELATIVE_HUMIDITY;
  event->timestamp = millis();

  getCapacitance(&capacitance);
  event->relative_humidity = calculateRelativeHumidity(capacitance);

  if (capacitance == 65536) {
    return false;  
  }

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void Chirp_Sensor_Unified::getSensor(sensor_t *sensor)
{
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "CHIRP", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _chirpSensorID;
  sensor->type        = SENSOR_TYPE_RELATIVE_HUMIDITY;
  sensor->min_delay   = 0;
  sensor->max_value   = 100.0;
  sensor->min_value   = 0.0;
  sensor->resolution  = 0.01;
}
