

/***************************************************
  Menu Definition
 ****************************************************/

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


// Menu callback functions

void on_item_selected(MenuItem* p_menu_item)
{
  display.setTextColor(RED, BLACK);
  display.setTextScale(1);
  display.setCursor(0, 56);
  display.print(p_menu_item->get_name());
  display.print(" selected");
}

void on_item1_selected(MenuItem* p_menu_item)
{
  on_item_selected(p_menu_item);

  display.setTextColor(RED, BLACK);
  display.setTextScale(1);
  display.setCursor(0, 18);
  display.print("Item1 Selected  ");
  //  delay(1500); // so we can look the result on the LCD
}

void on_item2_selected(MenuItem* p_menu_item)
{
  on_item_selected(p_menu_item);

  display.setTextColor(RED, BLACK);
  display.setTextScale(1);
  display.setCursor(0, 18);
  display.print("Item2 Selected  ");
  //  delay(1500); // so we can look the result on the LCD
}

void on_item3_selected(MenuItem* p_menu_item)
{
  on_item_selected(p_menu_item);

  display.setTextColor(RED, BLACK);
  display.setTextScale(1);
  display.setCursor(0, 18);
  display.print("Item3 Selected  ");
  //  delay(1500); // so we can look the result on the LCD
}

void on_display_moisture(MenuItem* p_menu_item)
{
  on_item_selected(p_menu_item);

  setDefaultViewMode(VIEWMODE_MOISTURE);
}
void on_display_light(MenuItem* p_menu_item)
{
  on_item_selected(p_menu_item);

  setDefaultViewMode(VIEWMODE_SENSORS);
}
void on_display_temperature(MenuItem* p_menu_item)
{
  on_item_selected(p_menu_item);

  setDefaultViewMode(VIEWMODE_ROTARY);
}
void on_display_outputs(MenuItem* p_menu_item)
{
  on_item_selected(p_menu_item);

  setDefaultViewMode(VIEWMODE_ROTARY);
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





/***************************************************
  Menu Render
 ****************************************************/

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
  display.setTextScale(1);

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

