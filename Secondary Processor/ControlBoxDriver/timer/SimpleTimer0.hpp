/*******************************************************************************
File Name : SimpleTimer0.hpp
Author    : Sean Collins
Purpose   : Defines a simple class interface for the Timer 0 module on an
            ATmega1280 processor.

 - Construct only one of these objects.
 
 - This timer provides a start function to begin the countdown.  It also
   provides an accessor that returns true if and only if the countdown has not
   finished yet.
   
 - Client code can test state of timer by calling "is_finished" method.

 - The range of possible timer durations is #### ms - #### ms.  
 
 - Client code must enable global interrupts by calling sei()

*******************************************************************************/
#ifndef SIMPLE_TIMER0_H
#define SIMPLE_TIMER0_H

#include <stdint-gcc.h>

class SimpleTimer0
{
public:
  SimpleTimer0 (uint16_t durationMS);
  
  void start ();

  void restart ();
  
  void set_duration (uint16_t);

  bool is_finished ();
};

#endif // SIMPLE_TIMER0_H
