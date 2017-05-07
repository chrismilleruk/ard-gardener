
/***************************************************
  This is a sketch to help water the plants.

  Chris Miller
 ****************************************************/

//Sketch uses 29172 bytes (94%) of program storage space. Maximum is 30720 bytes.
//Global variables use 1742 bytes (85%) of dynamic memory, leaving 306 bytes for local variables. Maximum is 2048 bytes.
//Low memory available, stability problems may occur.


/***************************************************
  Configuration Settings
 ****************************************************/

// Control output to the Serial Port.
#define SerialOn      true
#define SerialSpeed   57600
#define SerialSensors SerialOn && false
#define SerialInputs  SerialOn && true

#define MENU_TIMEOUT  10000
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


// SPI Hardware Pins: SPI: 10 (SS), 11 (MOSI), 12 (MISO), 13 (SCK).
#define rst  8
#define dc   9
#define ss   10
#define mosi 11 /* native h/w pins are fastest */
#define miso 12 /* native h/w pins are fastest */
#define sclk 13 /* native h/w pins are fastest */

// I2C Pins: A4 A5
// no definitions required.


/***************************************************
  Other Definitions
 ****************************************************/

// Color definitions
//#define BLACK           0x0000
//#define BLUE            0x001F
//#define RED             0xF800
//#define GREEN           0x07E0
//#define CYAN            0x07FF
//#define MAGENTA         0xF81F
//#define YELLOW          0xFFE0
//#define WHITE           0xFFFF
#define GRAY            0xCCCC


typedef enum
{
  VIEWMODE_SENSORS  = 0x00,
  VIEWMODE_ROTARY   = 0x01,
  VIEWMODE_MENU     = 0x02,
  VIEWMODE_OUTPUTS  = 0x03,
  VIEWMODE_MOISTURE = 0x04,
  VIEWMODE_TEMPERATURE = 0x05,
  VIEWMODE_LIGHT    = 0x06
} viewmode_type_t;

viewmode_type_t viewDefault = VIEWMODE_MOISTURE;
viewmode_type_t viewMode = VIEWMODE_SENSORS;
unsigned long viewModeChanged = 0;

/** struct hid_input is used to represent the input controls in a common format. */
typedef struct {
  boolean active;
  boolean changed;
  int8_t rotaryPos;
  int8_t rotaryDelta;
  boolean okPress;
  boolean backPress;
  boolean okLongPress;
  boolean backLongPress;
} hid_input_t;


/** struct ring_data contains the data for a ring. */
typedef struct {
  int val;
  int min;
  int max;
  uint16_t colorScheme;
  uint16_t backColor;
} ring_data_t;

typedef struct {
  float moisture;
  float pressure;
  float temperature;
  float light;
  unsigned long lastUpdated;
} sensor_values_t;


/***************************************************
  Library Includes
 ****************************************************/

// SPI Graphics
#include <SPI.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1331.h>
#include <SSD_13XX.h>

// HID Inputs
//#include <PinChangeInterrupt.h>
#include <RotaryEncoder.h>

// I2C Sensors
#include <Wire.h>
#include <Adafruit_MCP9808.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include "Chirp_Sensor_U.h"

// Menu
#include <MenuSystem.h>


/***************************************************
  Globals
 ****************************************************/

// Create the Display
// Option 1: use any pins but a little slower
//Adafruit_SSD1331 display = Adafruit_SSD1331(ss, dc, mosi, sclk, rst);
// Option 2: must use the hardware SPI pins
//Adafruit_SSD1331 display = Adafruit_SSD1331(ss, dc, rst);
// Option 3: use the Sumotoy SSD driver (fast)!
SSD_13XX display = SSD_13XX(ss, dc, rst);

// Create RotaryEncoder:
RotaryEncoder encoder(rotaryPin1, rotaryPin2);

// Create the sensor objects
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
Chirp_Sensor_Unified chirp = Chirp_Sensor_Unified(0x20);

// Menu variables
MenuSystem ms;


/***************************************************
  Setup & Loop
 ****************************************************/

void setup(void) {
  if (SerialOn) {
    Serial.begin(SerialSpeed);
    Serial.println(">>>");
    Serial.println("Let's look after the plants!");
  }

  // Setup rotary encoder pins.
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

  // Setup output pins
  digitalWrite(out1, HIGH);
  pinMode(out1, OUTPUT);
  digitalWrite(out2, HIGH);
  pinMode(out2, OUTPUT);
  digitalWrite(out3, HIGH);
  pinMode(out3, OUTPUT);
  digitalWrite(out4, HIGH);
  pinMode(out4, OUTPUT);
  digitalWrite(out5, HIGH);
  pinMode(out5, OUTPUT);
  digitalWrite(out6, HIGH);
  pinMode(out6, OUTPUT);

  // Initialise software
  menu_setup();

  // Initialise hardware
  initDisplay(false);
  initSensors(false);

  // Clear screen
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

      if (setViewMode(VIEWMODE_MENU)) {
        displayMenu(true);
      }
    }

    if (hid_inputs.backLongPress) {
      setViewMode(viewDefault);
    } else if (!hid_inputs.active && currentMillis - lastInputActivity > MENU_TIMEOUT) {
      setViewMode(viewDefault);
    }

    if (viewMode == VIEWMODE_ROTARY && hid_inputs.changed) {
      displayHidInputs(&hid_inputs);
    }

    if (viewMode == VIEWMODE_MENU && hid_inputs.changed) {
      if (hid_inputs.rotaryDelta > 0) {
        if (ms.next()) displayMenu(false);
      } else if (hid_inputs.rotaryDelta < 0) {
        if (ms.prev()) displayMenu(false);
      } else if (hid_inputs.backPress) {
        if (ms.back()) {
          displayMenu(true);
        } else {
          setViewMode(viewDefault);
        }
      } else if (hid_inputs.okPress) {
        int ptr1 = (int)ms.get_current_menu();
        ms.select();
        int ptr2 = (int)ms.get_current_menu();
        if (ptr1 != ptr2) {
          // Redraw if the action changed the current menu.
          displayMenu(true);
        }
      }

    }
  }

  // Sensor Sampling
  static sensor_values_t sensor_readings;
  boolean newData = false;
  unsigned long lastUpdated = sensor_readings.lastUpdated;

  static unsigned long lastSensorLoop = 0;
  if (currentMillis - lastSensorLoop > 1000) {
    lastSensorLoop = currentMillis;
    boolean sendToDisplay = (viewMode == VIEWMODE_SENSORS);

    if (sendToDisplay) heartbeat();
    
    sampleChirpSensor(&sensor_readings, sendToDisplay);
    sampleBCP180(&sensor_readings, sendToDisplay);
    sampleMCP9808(&sensor_readings, sendToDisplay);

    newData = (lastUpdated != sensor_readings.lastUpdated);
    
    if (sensor_readings.moisture < 10) {
      digitalWrite(out6, LOW);
    } else {
      digitalWrite(out6, HIGH);
    }
  }
  
  if (viewMode == VIEWMODE_MOISTURE) {
    displayRingMeter(&sensor_readings);
  }
}

bool setDefaultViewMode(viewmode_type_t newMode) {
  if (viewDefault != newMode) {
    viewDefault = newMode;
    setViewMode(newMode);
    return true;
  }
  return false;
}

bool setViewMode(viewmode_type_t newMode) {
  if (viewMode != newMode) {
    viewMode = newMode;
    viewModeChanged = millis();
    display.fillScreen(BLACK);
    return true;
  }
  return false;
}

// The Interrupt Service Routine for Pin Change Interrupt 1
// This routine will only be called on any signal change on A2 and A3: exactly where we need to check.
ISR(PCINT1_vect) {
  encoder.tick(); // just call tick() to check the state.
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
  static int x = display.width() - w ;

  int y = 0 + (pos * h);

  display.setCursor(x + border, y);
  display.setTextScale(textSize);
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

  const int t1x = x1 + 7;
  const int t1y = y1 + 7;
  const int t2x = t1x;
  const int t2y = y1 + h1 - 7;
  const int t3x = x1 + w1 - 7;
  const int t3y = y1 + (h1 / 2);

  const int p1x = x2 + 7;
  const int p1y = t1y;
  const int p1w = 12;
  const int p1h = t2y - t1y;
  const int p2x = x2 + w1 - 12 - 7;
  const int p2y = t1y;
  const int p2w = 12;
  const int p2h = t2y - t1y;


  // play
  display.fillScreen(BLACK);
  display.fillRoundRect(x1, y1, w1, h1, 8, WHITE);
  display.fillTriangle(t1x, t1y, t2x, t2y, t3x, t3y, RED);
  delay(500);
  // pause
  display.fillRoundRect(x2, y1, w1, h1, 8, WHITE);
  display.fillRoundRect(p1x, p1y, p1w, p1h, 5, GREEN);
  display.fillRoundRect(p2x, p2y, p2w, p2h, 5, GREEN);
  delay(500);
  // play color
  display.fillTriangle(t1x, t1y, t2x, t2y, t3x, t3y, BLUE);
  delay(50);
  // pause color
  display.fillRoundRect(p1x, p1y, p1w, p1h, 5, RED);
  display.fillRoundRect(p2x, p2y, p2w, p2h, 5, RED);
  // play color
  display.fillTriangle(t1x, t1y, t2x, t2y, t3x, t3y, GREEN);
}


