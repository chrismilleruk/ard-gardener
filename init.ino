
/***************************************************
  Setup Routines
 ****************************************************/


void initDisplay(boolean runQuickTest) {
  // Set up display
  display.begin();
  display.setRotation(2);

  if (runQuickTest) {
    // Run display checks
    if (SerialOn) {
      Serial.println("Quick Display Test");
    }

    uint16_t time = millis();
    display.fillScreen(RED);
    display.fillScreen(BLUE);
    display.fillScreen(GREEN);
    display.fillScreen(BLACK);
    time = millis() - time;

    if (SerialOn) {
      Serial.print("-- complete ");
      Serial.print(time, DEC);
      Serial.println("ms");
    }

    //    printTimeTaken(time);
    display.setCursor(5, 10);
    display.setTextColor(BLUE);
    display.setTextScale(2);
    display.print(time);
    display.print("ms");

    delay(600);
  }

  display.fillScreen(BLACK);
}

void initSensors(boolean showSummary) {

  /* Initialise the MCP9808 sensor */
  if (!tempsensor.begin()) {
    if (SerialOn) Serial.println("Couldn't find MCP9808!");

    display.fillScreen(BLACK);
    display.setTextScale(1);
    display.setTextColor(RED);
    display.setCursor(0, 0);
    display.print("Couldn't find MCP9808!");
    while (1);
  }

  /* Initialise the BCP180 sensor */
  if (!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    if (SerialOn) Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");

    display.fillScreen(BLACK);
    display.setTextScale(1);
    display.setTextColor(RED);
    display.setCursor(0, 0);
    display.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  if (!chirp.begin())
  {
    /* There was a problem detecting the Chirp */
    if (SerialOn) Serial.print("Ooops, no Chirp detected ... Check your wiring or I2C ADDR!");

    display.fillScreen(BLACK);
    display.setTextScale(1);
    display.setTextColor(RED);
    display.setCursor(0, 0);
    display.print("Ooops, no Chirp detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  if (showSummary) {
    // For universal sensors.
    sensor_t sensor;

    /* Show some BCP180 stats */
    bmp.getSensor(&sensor);
    display.fillScreen(BLACK);
    display.setCursor(0, 0);
    display.setTextScale(1);
    display.setTextColor(GRAY);
    display.print  ("Nom:"); display.println(sensor.name);
    display.print  ("Ver:"); display.println(sensor.version);
    display.print  ("ID: "); display.println(sensor.sensor_id);
    display.print  ("Max:"); display.print(sensor.max_value); display.println(" hPa");
    display.print  ("Min:"); display.print(sensor.min_value); display.println(" hPa");
    display.print  ("Res:"); display.print(sensor.resolution); display.println(" hPa");
    delay(1500);

    /* Show some Chirp stats */
    chirp.getSensor(&sensor);
    display.fillScreen(BLACK);
    display.setCursor(0, 0);
    display.setTextScale(1);
    display.setTextColor(GRAY);
    display.print  ("Nom:"); display.println(sensor.name);
    display.print  ("Ver:"); display.println(sensor.version);
    display.print  ("ID: "); display.println(sensor.sensor_id);
    display.print  ("Max:"); display.print(sensor.max_value); display.println("%");
    display.print  ("Min:"); display.print(sensor.min_value); display.println("%");
    display.print  ("Res:"); display.print(sensor.resolution); display.println("%");
    delay(1500);
  }

}


