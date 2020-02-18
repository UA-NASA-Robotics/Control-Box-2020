#include "ControlBoxEngine.hpp"

#define F_CPU 16000000UL
#include <util/delay.h>

int main ()
{
  _delay_ms(1000);
  ControlBoxEngine box;
  box.initialize();
  box.loop();
  return 0;
}