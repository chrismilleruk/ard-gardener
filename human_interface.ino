

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
  display.setTextScale(3);
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

