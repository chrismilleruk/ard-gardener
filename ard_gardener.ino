
/*************************************************** 
  This is a sketch to help water the plants.

  Chris Miller
 ****************************************************/

 
/*************************************************** 
  Configuration Settings
 ****************************************************/

// Control output to the Serial Port.
#define SerialOn      true
#define SerialSpeed   57600
#define SerialSensors SerialOn && false
#define SerialInputs  SerialOn && true

#define MENU_TIMEOUT  3000
#define LONG_PRESS    1000
  
/*************************************************** 
  Pin Definitions
 ****************************************************/

 
// Input Pins:
#define rotaryPin1  A2
#define rotaryPin2  A3
#define okBtnPin    A0
#define backBtnPin  A1


// Output Pins: 
#define outputCount 6
#define out1        2
#define out2        3
#define out3        4
#define out4        5
#define out5        6
#define out6        7
//#define out7        6
//#define out8        7


// SPI Hardware Pins: SPI: 10 (SS), 11 (MOSI), 12 (MISO), 13 (SCK). 
#define dc   8
#define rst  9
#define ss   10
#define mosi 11 // native h/w pins are fastest 
#define miso 12 // native h/w pins are fastest
#define sclk 13 // native h/w pins are fastest

// I2C Pins: A4 A5
// no definitions required.


/*************************************************** 
  Other Definitions
 ****************************************************/
 
// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF
#define GRAY            0xCCCC


typedef enum
{
  VIEWMODE_SENSORS  = 0x00,
  VIEWMODE_ROTARY   = 0x01,
  VIEWMODE_MENU     = 0x02
} viewmode_type_t;

viewmode_type_t viewMode = VIEWMODE_SENSORS;
unsigned long viewModeChanged = 0;

/** struct hid_input is used to represent the input controls in a common format. */
typedef struct {
    boolean active;
    boolean changed;
    int8_t rotaryPos;
    boolean okPress;
    boolean backPress;
    boolean okLongPress;
    boolean backLongPress;
} hid_input_t;



/*************************************************** 
  Library Includes
 ****************************************************/
 
// SPI Graphics
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>

// HID Inputs
//#include <PinChangeInterrupt.h>
#include <RotaryEncoder.h>

// I2C Sensors
#include <Wire.h>
#include <Adafruit_MCP9808.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include "Chirp_Sensor_U.h"



/*************************************************** 
  Globals
 ****************************************************/

// Create the Display
// Option 1: use any pins but a little slower
//Adafruit_SSD1331 display = Adafruit_SSD1331(ss, dc, mosi, sclk, rst);  
// Option 2: must use the hardware SPI pins 
Adafruit_SSD1331 display = Adafruit_SSD1331(ss, dc, rst);

// Create RotaryEncoder:
RotaryEncoder encoder(rotaryPin1, rotaryPin2);

// Create the sensor objects
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
Chirp_Sensor_Unified chirp = Chirp_Sensor_Unified(0x20);



/*************************************************** 
  Setup & Loop
 ****************************************************/
 
void setup(void) {
  if (SerialOn) {
    Serial.begin(SerialSpeed);
    Serial.println(">>>");
    Serial.println("Let's look after the plants!");
  }
  
  if (rotaryPin1 ^ rotaryPin2 == A2 ^ A3) {
    // Native Interrupts for pins A2 and A3
    PCICR |= (1 << PCIE1);    // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port C.
    PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);  // This enables the interrupt for pin 2 and 3 of Port C.
  } else {
    // Attach the new PinChangeInterrupt and enable event function below
    //  attachPCINT(digitalPinToPCINT(rotaryPin1), onPinChangeInterrupt, CHANGE);
    //  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(rotaryPin1), onPinChangeInterrupt, CHANGE);
  }
  pinMode(okBtnPin, INPUT_PULLUP);
  pinMode(backBtnPin, INPUT_PULLUP);
  
  initDisplay(true);
  initSensors(true);

  display.fillScreen(BLACK);
}

void loop() {
  unsigned long currentMillis = millis();

  // Read the rotary encoder every time
  encoder.tick();

  // Input handling
  static unsigned long lastInputLoop = 0;
  static unsigned long lastInputActivity = 0;
  if (currentMillis - lastInputLoop > 100) {
    lastInputLoop = currentMillis;

    hid_input_t hid_inputs;
    if (sampleHID(&hid_inputs)) {
      // Activity detected.
      lastInputActivity = currentMillis;
      setViewMode(VIEWMODE_ROTARY);
    }

    if (hid_inputs.backLongPress) {
      setViewMode(VIEWMODE_SENSORS);
    } else if (!hid_inputs.active && currentMillis - lastInputActivity > MENU_TIMEOUT) {
      setViewMode(VIEWMODE_SENSORS);
    }

    if (viewMode == VIEWMODE_ROTARY && hid_inputs.changed) {
      displayHidInputs(&hid_inputs);
    }
  }

  // Sensor Sampling
  static unsigned long lastSensorLoop = 0;
  if (currentMillis - lastSensorLoop > 1000) {
    lastSensorLoop = currentMillis;
    boolean sendToDisplay = (viewMode == VIEWMODE_SENSORS);

    heartbeat();
    sampleChirpSensor(sendToDisplay);
    sampleBCP180(sendToDisplay);
    sampleMCP9808(sendToDisplay);
  }
}

void setViewMode(viewmode_type_t newMode) {
  if (viewMode != newMode) {
    viewMode = newMode;
    viewModeChanged = millis();
    display.fillScreen(BLACK);
  }
}

//void onPinChangeInterrupt() {
//  encoder.tick();
//}
//

// The Interrupt Service Routine for Pin Change Interrupt 1
// This routine will only be called on any signal change on A2 and A3: exactly where we need to check.
ISR(PCINT1_vect) {
  encoder.tick(); // just call tick() to check the state.
}





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
    display.setTextSize(2);
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
    display.setTextSize(1);
    display.setTextColor(RED);
    display.setCursor(0,0);
    display.print("Couldn't find MCP9808!");
    while (1);
  }

  /* Initialise the BCP180 sensor */
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    if (SerialOn) Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    
    display.fillScreen(BLACK);
    display.setTextSize(1);
    display.setTextColor(RED);
    display.setCursor(0,0);
    display.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  if (!chirp.begin())
  {
    /* There was a problem detecting the Chirp */
    if (SerialOn) Serial.print("Ooops, no Chirp detected ... Check your wiring or I2C ADDR!");
    
    display.fillScreen(BLACK);
    display.setTextSize(1);
    display.setTextColor(RED);
    display.setCursor(0,0);
    display.print("Ooops, no Chirp detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  if (showSummary) {
    // For universal sensors.
    sensor_t sensor;
    
    /* Show some BCP180 stats */
    bmp.getSensor(&sensor);
    display.fillScreen(BLACK);
    display.setCursor(0,0);
    display.setTextSize(1);
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
    display.setCursor(0,0);
    display.setTextSize(1);
    display.setTextColor(GRAY);
    display.print  ("Nom:"); display.println(sensor.name);
    display.print  ("Ver:"); display.println(sensor.version);
    display.print  ("ID: "); display.println(sensor.sensor_id);
    display.print  ("Max:"); display.print(sensor.max_value); display.println(" hPa");
    display.print  ("Min:"); display.print(sensor.min_value); display.println(" hPa");
    display.print  ("Res:"); display.print(sensor.resolution); display.println(" hPa"); 
    delay(1500);
  }

}




/*************************************************** 
  Loop Routines (Human Interface)
 ****************************************************/

// Returns true if there is new HID activity.
boolean sampleHID(hid_input_t *hid_state) {
  // Local static variables.
  static char rotaryPosState = 0;
  static boolean okPressState = 0;
  static boolean okLongPressState = 0;
  static unsigned long okPressStart = 0;
  static boolean backPressState = 0;
  static boolean backLongPressState = 0;
  static unsigned long backPressStart = 0;

  // Local variables.
  int currentMillis = millis();
  boolean changed = false;
  int rotaryPos = encoder.getPosition();
  int okButton = digitalRead(okBtnPin);
  int backButton = digitalRead(backBtnPin);

  if (rotaryPosState != rotaryPos) {
    changed = true;
    rotaryPosState = rotaryPos;
  }
  
  hid_state->active = changed;

  processButtonPress(
    (okButton == LOW), currentMillis,
    &okPressState, &okPressStart, &okLongPressState, &changed);

  processButtonPress(
    (backButton == LOW), currentMillis,
    &backPressState, &backPressStart, &backLongPressState, &changed);

  hid_state->active |= okPressState | backPressState;
  hid_state->changed = changed;
  hid_state->rotaryPos = rotaryPosState;
  hid_state->okPress = okPressState;
  hid_state->backPress = backPressState;
  hid_state->okLongPress = okLongPressState;
  hid_state->backLongPress = backLongPressState;
  
  return hid_state->active;
}

void processButtonPress(boolean btnPressed, unsigned long currentMillis, 
  boolean *pressState, unsigned long *longPressStart, boolean *longPressState, boolean *changed)
{
  if (*pressState != btnPressed) {
    *changed = true;
    *pressState = btnPressed;
    *longPressStart = btnPressed ? currentMillis : 0;
  }
  boolean longPress = (btnPressed && currentMillis - *longPressStart > LONG_PRESS);
  if (*longPressState != longPress) {
    *changed = true;
    *longPressState = longPress;
  }
}

void displayHidInputs(hid_input_t *hid_inputs) {
      display.setCursor(0, 18);
      display.setTextColor(MAGENTA, BLACK);
      display.setTextSize(3);
      display.print(hid_inputs->rotaryPos);
      display.print(" ");

      if (hid_inputs->okLongPress) {
        plotSymbol(2, 7, WHITE, BLACK, false);
      } else if (hid_inputs->okPress) {
        plotSymbol(2, 7, BLUE, BLACK, false);
      } else {
        plotSymbol(2, 9, BLUE, BLACK, false);
      }
      
      if (hid_inputs->backLongPress) {
        plotSymbol(3, 7, WHITE, BLACK, false);
      } else if (hid_inputs->backPress) {
        plotSymbol(3, 7, BLUE, BLACK, false);
      } else {
        plotSymbol(3, 9, BLUE, BLACK, false);
      }
}

/*************************************************** 
  Loop Routines (Sensors)
 ****************************************************/
 
void sampleChirpSensor(boolean displaySensors) {
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
      display.setTextSize(1);
  
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
      display.setTextSize(1);
  
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
    display.setTextSize(1);
    display.setCursor(0, 48);display.print("Temp:"); 
    display.setCursor(48,48);display.print(c); display.write(9); display.print("C"); 
    display.setCursor(48,56);display.print(f); display.write(9); display.println("F");
  }
}

void heartbeat() {
  static boolean heartbeat_toggle = false;
  
  if (heartbeat_toggle) {
    plotSymbol(0, 3, WHITE, RED, true);
  } else {
    plotSymbol(0, 3, RED, BLACK, true);
  }

  heartbeat_toggle = !heartbeat_toggle;
}

void plotSymbol(char pos, char ch, uint16_t fgColor, uint16_t bgColor, boolean flash) {
  static int textSize = 2;
  static int border = 1 * textSize;
  static int w = 5 * textSize + border * 2;
  static int h = 7 * textSize + border;
  static int x = display.width() -w ;
  
  int y = 0 + (pos * h);

  display.setCursor(x + border, y);
  display.setTextSize(textSize);
  display.setTextWrap(false);

  if (flash) {
    display.fillRect(x, y, w + 4, h, WHITE);
  }

  display.fillRect(x, y, w, h, bgColor);
  display.setTextColor(fgColor);

  display.write(ch);
}


/*************************************************** 
  Notes - Not in use.
 ****************************************************/
 
void mediabuttons() {
  const int x = 0;
  const int y = 0;
  const int w = display.width();
  const int h = display.height();
  const int border = 2;
  const int gap = 2;

  const int x1 = x + border;
  const int x2 = (w / 2) + (gap / 2);
  const int y1 = y + border;
  const int w1 = (w / 2) - (gap / 2) - border;
  const int h1 = h - border * 2;

  const int t1x = x1+7;
  const int t1y = y1+7;
  const int t2x = t1x;
  const int t2y = y1+h1-7;
  const int t3x = x1+w1-7;
  const int t3y = y1+(h1/2);

  const int p1x = x2+7;
  const int p1y = t1y;
  const int p1w = 12;
  const int p1h = t2y-t1y;
  const int p2x = x2+w1-12-7;
  const int p2y = t1y;
  const int p2w = 12;
  const int p2h = t2y-t1y;

  
 // play
  display.fillScreen(BLACK);
  display.fillRoundRect(x1, y1, w1, h1, 8, WHITE);
  display.fillTriangle(t1x, t1y, t2x, t2y, t3x, t3y, RED);
  delay(500);
  // pause
  display.fillRoundRect(x2, y1, w1, h1, 8, WHITE);
  display.fillRoundRect(p1x,p1y,p1w,p1h, 5, GREEN);
  display.fillRoundRect(p2x,p2y,p2w,p2h, 5, GREEN);
  delay(500);
  // play color
  display.fillTriangle(t1x, t1y, t2x, t2y, t3x, t3y, BLUE);
  delay(50);
  // pause color
  display.fillRoundRect(p1x,p1y,p1w,p1h, 5, RED);
  display.fillRoundRect(p2x,p2y,p2w,p2h, 5, RED);
  // play color
  display.fillTriangle(t1x, t1y, t2x, t2y, t3x, t3y, GREEN);
}

