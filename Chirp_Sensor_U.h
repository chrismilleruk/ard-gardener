
#ifndef _CHIRP_H_
#define _CHIRP_H_

#if ARDUINO >= 100
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Adafruit_Sensor.h>

#ifdef __AVR_ATtiny85__
  #include "TinyWireM.h"
  #define Wire TinyWireM
#else
  #include <Wire.h>
#endif

#define CHIRP_ADDR_DEFAULT    (0x20)

enum
{
  CHIRP_GET_CAPACITANCE = 0x00,
  CHIRP_SET_ADDRESS     = 0x01,
  CHIRP_GET_ADDRESS     = 0x02,
  CHIRP_MEASURE_LIGHT   = 0x03,
  CHIRP_GET_LIGHT       = 0x04,
  CHIRP_GET_TEMPERATURE = 0x05,
  CHIRP_RESET           = 0x06,
  CHIRP_GET_VERSION     = 0x07,
  CHIRP_SLEEP           = 0x08,
  CHIRP_GET_BUSY        = 0x09
};

class Chirp_Sensor_Unified : public Adafruit_Sensor {
 public:
  Chirp_Sensor_Unified(uint8_t addr, int32_t sensorID = -1);
  boolean begin(void);
  
  /* Chirp Functions */
  void getCapacitance(uint16_t *capacitance);
  float calculateRelativeHumidity(uint16_t capacitance);
  void getTemperature(float *temperature);
  float calculateCelsius(uint16_t temperatureSensor);
  void getLight (float *light);
  float calculateLux(uint16_t lightSensor);

  /* Unified Sensor API Functions */
  bool getEvent(sensors_event_t*);
  void getSensor(sensor_t*);

 private:
  int8_t _addr;
  boolean _chirpInitialised;
  int32_t _chirpSensorID;

  void     enable (void);
  void     disable (void);
  void     write8 (uint8_t reg, uint32_t value);
  void     write8 (uint8_t reg);
  uint8_t  read8 (uint8_t reg);
  uint16_t read16 (uint8_t reg);
  void     getData (uint16_t *broadband, uint16_t *ir);
};
#endif
