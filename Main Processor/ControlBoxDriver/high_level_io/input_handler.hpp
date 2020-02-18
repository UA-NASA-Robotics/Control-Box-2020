#ifndef INPUT_HANDLER_HPP
#define INPUT_HANDLER_HPP

#include "../digital_io/expander.hpp"
#include "../digital_io/expander_pin.hpp"
#include "../digital_io/digital_pin.hpp"
#include "../analog_input/analog_input.hpp"
#include "../memory/memory.hpp"

class InputHandler
{
public:
  void initialize (Memory *);
  void poll ();
  void clear_previous_inputs ();
private:
  void initialize_expanders ();
  void initialize_push_buttons ();
  void initialize_arcade_buttons ();
  void initialize_joysticks ();
  void initialize_potentiometers ();
  void poll_arcade_buttons ();
  void poll_push_buttons ();
  void poll_analog_inputs ();
private:
  Memory * memory;
  Expander expander_left;
  Expander expander_right;
  ExpanderPin push_buttons [16];
  DigitalPin arcade_buttons [9];
  AnalogInput joystick_left_x;
  AnalogInput joystick_left_y;
  AnalogInput joystick_right_x;
  AnalogInput joystick_right_y;
  AnalogInput potentiometer_left;
  AnalogInput potentiometer_right;
};

#endif // INPUT_HANDLER_HPP