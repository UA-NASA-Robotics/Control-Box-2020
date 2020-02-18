#ifndef EXPANDER_PIN_HPP
#define EXPANDER_PIN_HPP
#include "expander.hpp"
#include "digital_io_params.hpp"
#include <stdint-gcc.h>

class ExpanderPin
{
public:
  void initialize (Expander *, Port, Pin, Mode);
  bool read ();
  void write (bool);
private:
  void set_mode (Mode);
  Port port;
  Pin pin;
  Mode mode;
  Expander * expander;
};

#endif // EXPANDER_PIN_HPP
