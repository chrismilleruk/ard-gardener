
/***************************************************
  Loop Routines (Sensors)
 ****************************************************/

float sampleChirpSensor(boolean displaySensors) {
  sensors_event_t event;
  chirp.getEvent(&event);

  if (event.relative_humidity) {

    float chirpTemperature;
    chirp.getTemperature(&chirpTemperature);

    float light;
    chirp.getLight(&light);

    if (SerialSensors) {
      Serial.print("Chirp:\t");
      Serial.print(event.relative_humidity);
      Serial.print("%\t");
      Serial.print(chirpTemperature);
      Serial.print("*C\t");
      Serial.print(light);
      Serial.println("lux");
    }

    if (displaySensors) {
      display.setTextColor(RED, BLACK);
      display.setTextScale(1);

      display.setCursor(0, 0);

      /* Display humidity */
      display.print("Hum:");
      display.print(event.relative_humidity);
      display.println("%");

      /* Display temperature in C */
      display.print("Temp:");
      display.print(chirpTemperature);
      display.println("C");

      /* Display temperature in C */
      display.print("Lux:");
      display.print(light);
      display.println("");
    }
  }

  return event.relative_humidity;
}

void sampleBCP180(boolean displaySensors) {

  // BCP180
  sensors_event_t event;
  bmp.getEvent(&event);

  /* Display the results (barometric pressure is measure in hPa) */
  if (event.pressure)
  {
    if (displaySensors) {
      display.setTextColor(GRAY, BLACK);
      display.setTextScale(1);

      display.setCursor(0, 24);

      /* Display atmospheric pressue in hPa */
      display.print("Pres:");
      display.print(event.pressure);
      display.println(" hPa");
    }

    /* First we get the current temperature from the BMP085 */
    float temperature;
    bmp.getTemperature(&temperature);

    /* Then convert the atmospheric pressure, and SLP to altitude         */
    /* Update this next line with the current SLP for better results      */
    float seaLevelPressure =  1011.3; //SENSORS_PRESSURE_SEALEVELHPA;

    float altitude = bmp.pressureToAltitude(seaLevelPressure, event.pressure);

    if (displaySensors) {
      display.print("Temp:");
      display.print(temperature);
      display.println(" C");

      display.print("Alt: ");
      display.print(altitude);
      display.println(" m");
    }

    if (SerialSensors) {
      Serial.print("BCP180:\t");
      Serial.print(event.pressure); Serial.print("hPa\t");
      Serial.print(temperature); Serial.println("*C");
    }
  }

}

void sampleMCP9808(boolean displaySensors) {
  // MCP9808
  // Read and print out the temperature, then convert to *F
  float c = tempsensor.readTempC();
  float f = c * 9.0 / 5.0 + 32;

  if (SerialSensors) {
    Serial.print("MCP9808:\t");
    Serial.print(c); Serial.print("*C\t");
    Serial.print(f); Serial.println("*F");
  }

  if (displaySensors) {
    display.setTextColor(YELLOW, BLACK);
    display.setTextScale(1);
    display.setCursor(0, 48); display.print("Temp:");
    display.setCursor(48, 48); display.print(c); display.write(9); display.print("C");
    display.setCursor(48, 56); display.print(f); display.write(9); display.println("F");
  }
}

