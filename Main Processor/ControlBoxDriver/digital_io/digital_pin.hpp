#ifndef DIGITAL_PIN_HPP
#define DIGITAL_PIN_HPP
#include <stdbool.h>
#include "digital_io_params.hpp"

class DigitalPin
{
public:
  void initialize (Port, Pin, Mode);
  void write (bool);
  void toggle ();
  bool read ();
  bool is_high ();
  bool is_low ();
private:
  Port port;
  Pin  pin;
  Mode mode;
};

#endif // DIGITAL_PIN_HPP
