

/***************************************************
  Ring Meter Mode
 ****************************************************/

void displayRingMeter(ring_data_t *ring1) {
  unsigned long ms = millis();

  static unsigned long d1 = 7000;
  static unsigned long t1 = ms + d1 + 1;
  if (ms - t1 > d1) {
    t1 = ms;

    display.fillRect(0, 0, 0x60, 10, BLACK, BLACK);
    display.fillRect(0x20, 10, 0x20, 6, BLACK);

    display.drawFastHLine(0,    10, 0x20, 0x528A); //0xD6BA);
    display.drawFastHLine(0x20, 16, 0x20, 0x528A); //0xD6BA);
    display.drawFastHLine(0x40, 10, 0x20, 0x528A); //0xD6BA);
    display.drawFastVLine(0x1F, 10, 7,    0x528A); //0xD6BA);
    display.drawFastVLine(0x40, 10, 7,    0x528A); //0xD6BA);

    display.setCursor(0, 0);
    display.setTextColor(WHITE);
    display.setTextScale(1);
    display.print("Moisture");
  }

  static unsigned long t2 = ms;
  if (ms - t2 > 500) {
    t2 = ms;

    // Outer Ring
    int r = 0x60 / 2 - 1;
    display.ringMeter(ring1->val, ring1->min, ring1->max,
                      0x60 / 2 - r, 0x40 - r, r,
                      ring1->colorScheme, ring1->backColor, 90);
    // lightgray 0xCE59
    // darkgray 0x528A

    display.fillRect(0x20, 0, 0x20, 16, BLACK);

    display.setCursor(CENTER, 0);
    display.setTextColor(0x528A, BLACK);
    display.setTextScale(2);
    display.print(ring1->val);

  }


  static unsigned long t3 = ms;
  if (ms - t3 > 30) {
    t3 = ms;

    // Inner Ring
    int r = 15;
    //    int pos = map(ms - t1, 0, d1, 0, 0xff);
    //    uint16_t color = tft.colorInterpolation(0, 0xff, 0, 0xff, 0, 0, pos, 0xff);

    display.ringMeter(ms - t1, d1, 0,
                      0x60 / 2 - r, 0x40 - 2 * r, r,
                      GREEN, RED);
  }

}


