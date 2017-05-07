

/***************************************************
  Ring Meter Mode
 ****************************************************/
void displayRingMeter(sensor_values_t *sensors) {
  displayRingMeter(sensors->moisture, sensors->pressure, sensors->temperature, sensors->light);
}

void displayRingMeter(float moisture, float pressure, float temperature, float light) {
  static int moisture_threshold = 35;
  
  // lightgray 0xCE59
  // darkgray 0x528A
  // gray 15% 0x3186
  
  static ring_data_t moisture_ring;
  moisture_ring.val = (int)moisture;
  moisture_ring.min = 0;
  moisture_ring.max = 100;
  moisture_ring.colorScheme = BLUE;
  moisture_ring.backColor = moisture < moisture_threshold ? RED : BLACK;

  static ring_data_t pressure_ring;
  pressure_ring.val = (int)pressure;
  pressure_ring.min = 950;
  pressure_ring.max = 1050;
  pressure_ring.colorScheme = 7;
  pressure_ring.backColor = BLACK;

  static ring_data_t temp_ring;
  temp_ring.val = (int)temperature;
  temp_ring.min = 15;
  temp_ring.max = 25;
  temp_ring.colorScheme = 3;
  temp_ring.backColor = 0x3186;

  static ring_data_t light_ring;
  light_ring.val = (int)light;
  light_ring.min = 0;
  light_ring.max = 300;
  light_ring.colorScheme = 10;
  light_ring.backColor = 0x3186;

  unsigned long ms = millis();

  static unsigned long d1 = 7000;
  static unsigned long t1 = ms + d1 + 1;
  if (ms - t1 > d1) {
    t1 = ms;

    display.drawFastHLine(0,    10, 0x20, 0x528A); //0xD6BA);
    display.drawFastHLine(0x20, 16, 0x20, 0x528A); //0xD6BA);
    display.drawFastHLine(0x40, 10, 0x20, 0x528A); //0xD6BA);
    display.drawFastVLine(0x1F, 10, 7,    0x528A); //0xD6BA);
    display.drawFastVLine(0x40, 10, 7,    0x528A); //0xD6BA);
    
    displayRingLabels(moisture_ring.val, light_ring.val, temperature, (int)pressure);

  }

  static unsigned long t2 = ms;
  if (ms - t2 > 2000) {
    t2 = ms;

    displayRingMeter(&pressure_ring, &light_ring, &temp_ring, &moisture_ring);
//  }
//  
//  static unsigned long t3 = ms;
//  if (ms - t3 > 2000) {
//    t3 = ms;

    displayRingLabels(moisture_ring.val, light_ring.val, temperature, (int)pressure);
  }
}

void displayRingLabels(int moisture, int lux, float temp, int pressure) {

    static float previous_temp;
    if (previous_temp != temp) {
      previous_temp = temp;
      
      display.fillRect(0, 0, 0x20, 10, BLACK, BLACK);
      
      uint16_t color = display.gradient(map(temp, 18, 24, 0, 127));
      display.setCursor(0, 0);
      display.setTextColor(color, BLACK);
      display.setTextScale(1);
      display.print(temp);
      display.print("'C");
    }


    static int previous_lux;
    if (previous_lux != lux) {
      previous_lux = lux;
      
      display.fillRect(0x40, 0, 0x20, 10, BLACK, BLACK);
  
      display.setCursor(0x40, 0);
      display.setTextColor(YELLOW, BLACK);
      display.setTextScale(1);
      display.print(lux);
      display.print("lux");
    }

    static int previous_pressure;
    if (previous_pressure != pressure) {
      previous_pressure = pressure;
      
      display.fillRect(0x40, 12, 0x20, 10, BLACK, BLACK);
  
      uint16_t color = display.gradient(map(pressure, 950, 1050, 35, 127));
      display.setCursor(0x40, 12);
      display.setTextColor(color, BLACK);
      display.setTextScale(1);
      display.print(pressure);
      display.print("hPa");
    }


    static int previous_moisture;
    if (previous_moisture != moisture) {
      previous_moisture = moisture;
      
      display.fillRect(0x20, 0, 0x20, 16, BLACK);
          
      display.setCursor(CENTER, 0);
      display.setTextColor(BLUE, BLACK);
      display.setTextScale(2);
      display.print(moisture);
    }

}

void displayRingMeter(ring_data_t *ring1, ring_data_t *ring2, ring_data_t *ring3, ring_data_t *ring4) {
    int gap = 5;
    // Outer Ring
    int r = 0x60 / 2 - 1;
    int w = 5;
    ringMeter(ring1->val, ring1->min, ring1->max,
              0x60 / 2 - r, 0x40 - r, r, w,
              ring1->colorScheme, ring1->backColor, 90, 5);


    // Middle Ring
    r -= w + gap;
//    r = 0x20;
//    w = r / 5;
    ringMeter(ring2->val, ring2->min, ring2->max,
              0x60 / 2 - r, 0x40 - r, r, w,
              ring2->colorScheme, ring2->backColor, 90, 5);



    // Inner Ring
    r -= w + gap;
//    r = 0x12;
//    w = r / 4;
    ringMeter(ring3->val, ring3->min, ring3->max,
              0x60 / 2 - r, 0x40 - r, r, w,
              ring3->colorScheme, ring3->backColor, 90, 5);


    // Inner Pie
    r -= w + gap;
//    r = 0x12;
    w = r;
    ringMeter(ring4->val, ring4->min, ring4->max,
              0x60 / 2 - r, 0x40 - r, r, w,
              ring4->colorScheme, ring4->backColor, 90, 5);


}



/**************************************************************************/
/*!
   Sketch uses 29230 bytes (95%) of program storage space. Maximum is 30720 bytes.
  Global variables use 1750 bytes (85%) of dynamic memory, leaving 298 bytes for local variables. Maximum is 2048 bytes.

      ringMeter
    (adapted from Alan Senior (thanks man!))
    from my RA8875 library
    it create a ring meter with a lot of personalizations,
    it return the width of the gauge so you can use this value
    for positioning other gauges near the one just created easily
    Parameters:
    val:  your value
    minV: the minimum value possible
    maxV: the max value possible
    x:    the position on x axis
    y:    the position on y axis
    r:    the radius of the gauge (minimum 20)
    units: a text that shows the units, if "none" all text will be avoided
    scheme:0...7 or 16 bit color (not BLACK or WHITE)
    0:red
    1:green
    2:blue
    3:blue->red
    4:green->red
    5:red->green
    6:red->green->blue
    7:cyan->green->red
    8:black->white linear interpolation
    9:violet->yellow linear interpolation
    10:black->yellow linear interpolation
    or
      RGB565 color (not BLACK or WHITE)
    backSegColor: the color of the segments not active (default BLACK)
    angle:    90 -> 180 (the shape of the meter, 90:halfway, 180:full round, 150:default)
    inc:      5...20 (5:solid, 20:sparse divisions, default:10)
*/
/**************************************************************************/
void ringMeter(int val, int minV, int maxV, 
               uint8_t x, uint8_t y, uint8_t r, uint8_t w, 
               uint16_t colorScheme, uint16_t backSegColor, 
               int angle, uint8_t inc)
{
  if (inc < 5) inc = 5;
  if (inc > 20) inc = 20;
  if (r < 10) r = 20;
  if (angle < 90) angle = 90;
  if (angle > 180) angle = 180;
  int i;
  int curAngle = map(val, minV, maxV, -angle, angle);
  uint16_t colour;
  x += r;
  y += r;   // Calculate coords of centre of ring
//  uint8_t w = r / 6;    // Width of outer ring is 1/4 of radius
  const uint8_t seg = 5; // Segments are 5 degrees wide = 60 segments for 300 degrees
  // Draw colour blocks every inc degrees
  for (i = angle; i > curAngle; i -= inc) {
    // Calculate pair of coordinates for segment start
    float xStart = cos((i - 90) * 0.0174532925);
    float yStart = sin((i - 90) * 0.0174532925);
    uint8_t x0 = xStart * (r - w) + x;
    uint8_t y0 = yStart * (r - w) + y;
    uint8_t x1 = xStart * r + x;
    uint8_t y1 = yStart * r + y;

    // Calculate pair of coordinates for segment end
    float xEnd = cos((i + seg - 90) * 0.0174532925);
    float yEnd = sin((i + seg - 90) * 0.0174532925);
    uint8_t x2 = xEnd * (r - w) + x;
    uint8_t y2 = yEnd * (r - w) + y;
    uint8_t x3 = xEnd * r + x;
    uint8_t y3 = yEnd * r + y;

    display.fillQuad(x0, y0, x1, y1, x2, y2, x3, y3, backSegColor, false);
  }
  for (i = -angle; i < curAngle; i += inc) {
    colour = BLACK;
    switch (colorScheme) {
      case 0:
        colour = RED;
        break; // Fixed colour
      case 1:
        colour = GREEN;
        break; // Fixed colour
      case 2:
        colour = BLUE;
        break; // Fixed colour
      case 3:
        colour = display.gradient(map(i, -angle, angle, 0, 127));
        break; // Full spectrum blue to red
      case 4:
        colour = display.gradient(map(i, -angle, angle, 63, 127));
        break; // Green to red (high temperature etc)
      case 5:
        colour = display.gradient(map(i, -angle, angle, 127, 63));
        break; // Red to green (low battery etc)
      case 6:
        colour = display.gradient(map(i, -angle, angle, 127, 0));
        break; // Red to blue (air cond reverse)
      case 7:
        colour = display.gradient(map(i, -angle, angle, 35, 127));
        break; // cyan to red
      case 8:
        colour = colorInterpolation(0, 0, 0, 255, 255, 255, map(i, -angle, angle, 0, angle), angle);
        break; // black to white
      case 9:
        colour = colorInterpolation(0x80, 0, 0xC0, 0xFF, 0xFF, 0, map(i, -angle, angle, 0, angle), angle);
        break; // violet to yellow
      case 10:
        colour = colorInterpolation(0x22, 0x22, 0, 0xFF, 0xFF, 0, map(i, -angle, angle, 0, angle), angle);
        break; // black to yellow
      default:
        if (colorScheme > 9) {
          colour = colorScheme;
        } else {
          colour = BLUE;
        }
        break; // Fixed colour
    }
    // Calculate pair of coordinates for segment start
    float xStart = cos((i - 90) * 0.0174532925);
    float yStart = sin((i - 90) * 0.0174532925);
    uint8_t x0 = xStart * (r - w) + x;
    uint8_t y0 = yStart * (r - w) + y;
    uint8_t x1 = xStart * r + x;
    uint8_t y1 = yStart * r + y;

    // Calculate pair of coordinates for segment end
    float xEnd = cos((i + seg - 90) * 0.0174532925);
    float yEnd = sin((i + seg - 90) * 0.0174532925);
    uint8_t x2 = xEnd * (r - w) + x;
    uint8_t y2 = yEnd * (r - w) + y;
    uint8_t x3 = xEnd * r + x;
    uint8_t y3 = yEnd * r + y;

    if (i < curAngle) { // Fill in coloured segments with 2 triangles
      display.fillQuad(x0, y0, x1, y1, x2, y2, x3, y3, colour, false);
    }
  }

}


uint16_t colorInterpolation(uint8_t r1,uint8_t g1,uint8_t b1,uint8_t r2,uint8_t g2,uint8_t b2,uint16_t pos,uint16_t div)
{
    if (pos == 0) return display.Color565(r1,g1,b1);
    if (pos >= div) return display.Color565(r2,g2,b2);
  float pos2 = (float)pos/div;
  return display.Color565(
        (uint8_t)(map(pos, 0, div, r1, r2)),
        (uint8_t)(map(pos, 0, div, g1, g2)),
        (uint8_t)(map(pos, 0, div, b1, b2))
//        (uint8_t)(((1.0 - pos2) * r1) + (pos2 * r2)),
//        (uint8_t)(((1.0 - pos2) * g1) + (pos2 * g2)),
//        (uint8_t)(((1.0 - pos2) * b1) + (pos2 * b2))
  );
}

