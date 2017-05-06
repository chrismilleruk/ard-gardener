
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

#define MENU_TIMEOUT  6000
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
#define dc   9
#define rst  8
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
  VIEWMODE_MENU     = 0x02,
  VIEWMODE_OUTPUTS  = 0x03,
  VIEWMODE_MOISTURE = 0x04,
  VIEWMODE_TEMPERATURE = 0x05,
  VIEWMODE_LIGHT    = 0x06
} viewmode_type_t;

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

// Menu
#include <MenuSystem.h>


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
      setViewMode(VIEWMODE_SENSORS);
    } else if (!hid_inputs.active && currentMillis - lastInputActivity > MENU_TIMEOUT) {
      setViewMode(VIEWMODE_SENSORS);
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
        if (ms.back()) displayMenu(true);
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
  static unsigned long lastSensorLoop = 0;
  if (currentMillis - lastSensorLoop > 1000) {
    lastSensorLoop = currentMillis;
    boolean sendToDisplay = (viewMode == VIEWMODE_SENSORS);

    heartbeat();
    float moisture = sampleChirpSensor(sendToDisplay);
    sampleBCP180(sendToDisplay);
    sampleMCP9808(sendToDisplay);

    if (moisture < 10) {
      digitalWrite(out6, LOW);
    } else {
      digitalWrite(out6, HIGH);
    }
  }
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
    display.setTextSize(1);
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
    display.setTextSize(1);
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
    display.setCursor(0, 0);
    display.setTextSize(1);
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




/***************************************************
  Loop Routines (Human Interface)
 ****************************************************/

// Returns true if there is new Human Interface Device activity.
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

  hid_state->rotaryDelta = 0;
  if (rotaryPosState != rotaryPos) {
    changed = true;
    hid_state->rotaryDelta = rotaryPos - rotaryPosState;
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
                        boolean *pressState, unsigned long *longPressStart,
                        boolean *longPressState, boolean *changed)
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
    display.setCursor(0, 48); display.print("Temp:");
    display.setCursor(48, 48); display.print(c); display.write(9); display.print("C");
    display.setCursor(48, 56); display.print(f); display.write(9); display.println("F");
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
  static int x = display.width() - w ;

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


void menu_setup() {

  static Menu mm("Settings");
  static MenuItem mm_mi1("Demo Item 1");
  static MenuItem mm_mi2("Demo Item 2");
  static MenuItem mm_mi3("Demo Item 3");
  mm.add_item(&mm_mi1, &on_item1_selected);
  mm.add_item(&mm_mi2, &on_item2_selected);
  mm.add_item(&mm_mi3, &on_item3_selected);

  static Menu mu1("Display");
  static MenuItem mu1_mi1("Moisture");
  static MenuItem mu1_mi2("Light");
  static MenuItem mu1_mi3("Temperature");
  static MenuItem mu1_mi4("Outputs");
  mm.add_menu(&mu1);
  mu1.add_item(&mu1_mi1, &on_display_moisture);
  mu1.add_item(&mu1_mi2, &on_display_light);
  mu1.add_item(&mu1_mi3, &on_display_temperature);
  mu1.add_item(&mu1_mi4, &on_display_outputs);

  static Menu mu2("Test Outputs");
  static MenuItem mu2_mi1("Output 1");
  static MenuItem mu2_mi2("Output 2");
  static MenuItem mu2_mi3("Output 3");
  static MenuItem mu2_mi4("Output 4");
  static MenuItem mu2_mi5("Output 5");
  static MenuItem mu2_mi6("Output 6");
  mm.add_menu(&mu2);
  mu2.add_item(&mu2_mi1, &on_test_output1);
  mu2.add_item(&mu2_mi2, &on_test_output2);
  mu2.add_item(&mu2_mi3, &on_test_output3);
  mu2.add_item(&mu2_mi4, &on_test_output4);
  mu2.add_item(&mu2_mi5, &on_test_output5);
  mu2.add_item(&mu2_mi6, &on_test_output6);

  // Menu setup
  ms.set_root_menu(&mm);
  Serial.println("Menu initialised.");
}

void displayMenu(bool redraw) {
  static const byte lineHeight = 9;
  static const byte lineSelected = 3;
  static const byte numBefore = lineSelected - 2;
  static const byte numAfter = 5 - lineSelected;

  static byte previousIndex = -1;

  // Display the menu
  Menu const* cp_menu = ms.get_current_menu();
  Menu const* cp_parent = cp_menu->get_parent();
  MenuComponent const* cp_menu_sel = cp_menu->get_selected();
  byte cp_menu_sel_idx = cp_menu->get_cur_menu_component_num();
  byte cp_menu_num_items = cp_menu->get_num_menu_components();

  Serial.print(cp_menu->get_name());
  Serial.print("-");
  Serial.print(cp_menu_sel->get_name());
  Serial.print("  -  ");
  Serial.println(redraw);

  display.setTextColor(BLUE, BLACK);
  display.setTextSize(1);

  if (redraw) {

    display.fillScreen(BLACK);

    // Menu title
    display.setTextColor(BLUE, BLACK);
    display.setCursor(0, lineHeight * 1);
    display.println(cp_menu->get_name());

    // Parent menu.
    display.setTextColor(GRAY, BLACK);
    display.setCursor(0, lineHeight * 0);
    if (cp_parent != NULL) {
      display.print("<<");
      display.println(cp_parent->get_name());
    } else {
      display.println("               ");
    }
  }

  if (redraw || previousIndex != cp_menu_sel_idx) {
    previousIndex = cp_menu_sel_idx;

    // Selected item.
    display.setTextColor(BLUE, BLACK);
    display.setCursor(0, lineHeight * lineSelected);
    display.print(cp_menu->get_selected()->get_name());
    display.println("               ");

    // Switch to Gray pen.
    display.setTextColor(GRAY, BLACK);

    // Preceding items.
    int lineOffset = lineSelected - cp_menu_sel_idx;
    for (int i = cp_menu_sel_idx - numBefore; i < cp_menu_sel_idx; ++i) {
      display.setCursor(0, lineHeight * (lineOffset + i));
      if (i >= 0) {
        MenuComponent const* cp_m_comp = cp_menu->get_menu_component(i);
        display.print(cp_m_comp->get_name());
      }
      display.println("               ");
    }

    // Following items.
    for (int i = cp_menu_sel_idx + 1; i <= cp_menu_sel_idx + numAfter; ++i) {
      display.setCursor(0, lineHeight * (lineOffset + i));
      if (i < cp_menu_num_items) {
        MenuComponent const* cp_m_comp = cp_menu->get_menu_component(i);
        display.print(cp_m_comp->get_name());
      }
      display.println("               ");
    }

  }

}


// Menu callback functions

void on_item_selected(MenuItem* p_menu_item)
{
  display.setTextColor(RED, BLACK);
  display.setTextSize(1);
  display.setCursor(0, 56);
  display.print(p_menu_item->get_name());
  display.print(" selected");
}

void on_item1_selected(MenuItem* p_menu_item)
{
  on_item_selected(p_menu_item);

  display.setTextColor(RED, BLACK);
  display.setTextSize(1);
  display.setCursor(0, 18);
  display.print("Item1 Selected  ");
  //  delay(1500); // so we can look the result on the LCD
}

void on_item2_selected(MenuItem* p_menu_item)
{
  on_item_selected(p_menu_item);

  display.setTextColor(RED, BLACK);
  display.setTextSize(1);
  display.setCursor(0, 18);
  display.print("Item2 Selected  ");
  //  delay(1500); // so we can look the result on the LCD
}

void on_item3_selected(MenuItem* p_menu_item)
{
  on_item_selected(p_menu_item);

  display.setTextColor(RED, BLACK);
  display.setTextSize(1);
  display.setCursor(0, 18);
  display.print("Item3 Selected  ");
  //  delay(1500); // so we can look the result on the LCD
}

void on_display_moisture(MenuItem* p_menu_item)
{
  on_item_selected(p_menu_item);

  setViewMode(VIEWMODE_ROTARY);
}
void on_display_light(MenuItem* p_menu_item)
{
  on_item_selected(p_menu_item);

  setViewMode(VIEWMODE_ROTARY);
}
void on_display_temperature(MenuItem* p_menu_item)
{
  on_item_selected(p_menu_item);

  setViewMode(VIEWMODE_ROTARY);
}
void on_display_outputs(MenuItem* p_menu_item)
{
  on_item_selected(p_menu_item);

  setViewMode(VIEWMODE_ROTARY);
}

void on_test_output1(MenuItem* p_menu_item)
{
  on_item_selected(p_menu_item);

  digitalWrite(out1, !digitalRead(out1));
}
void on_test_output2(MenuItem* p_menu_item)
{
  on_item_selected(p_menu_item);

  digitalWrite(out2, !digitalRead(out2));
}
void on_test_output3(MenuItem* p_menu_item)
{
  on_item_selected(p_menu_item);

  digitalWrite(out3, !digitalRead(out3));
}
void on_test_output4(MenuItem* p_menu_item)
{
  on_item_selected(p_menu_item);

  digitalWrite(out4, !digitalRead(out4));
}
void on_test_output5(MenuItem* p_menu_item)
{
  on_item_selected(p_menu_item);

  digitalWrite(out5, !digitalRead(out5));
}
void on_test_output6(MenuItem* p_menu_item)
{
  on_item_selected(p_menu_item);

  digitalWrite(out6, !digitalRead(out6));
}

