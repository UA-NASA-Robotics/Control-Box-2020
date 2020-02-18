#ifndef OUTPUT_HANDLER_HPP
#define OUTPUT_HANDLER_HPP

#include "../memory/memory.hpp"
#include "../screen/touch_screen.hpp"
#include "../communications/request.hpp"
#include "../digital_io/expander.hpp"
#include "../digital_io/expander_pin.hpp"
#include "../digital_io/digital_pin.hpp"

class OutputHandler
{
public:
  void initialize (Memory *);
  void startup ();
  void refresh ();
private:
  void initialize_expanders ();
  void initialize_push_button_leds ();
  void initialize_panel_leds ();
  void refresh_push_button_leds ();
  void refresh_panel_leds ();
  void turn_off_push_button_leds ();
  void spiral ();
  void drop ();
private:
  Memory * memory;
  TouchScreen screen;
  Expander expander_left;
  Expander expander_right;
  ExpanderPin push_button_leds [NUM_PUSH_BUTTONS];
  DigitalPin panel_leds [8];
  //SimpleTimer1 timer;
};

#endif // OUTPUT_HANDLER_HPP