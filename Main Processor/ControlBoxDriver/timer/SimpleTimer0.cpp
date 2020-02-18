/*******************************************************************************
File Name : SimpleTimer0.cpp
Author    : Sean Collins
Purpose   : Implements the SimpleTimer0 interface using functions from the
            C-style "timer0.h" library.

 - Be warned that this module uses some static global variables. "static" means
   (among other things) that the variables are only visible within this 
   translation unit.

 - In this case, the "class" construct is not providing encapsulation (because
   I violated encapsulation by storing the state of the module in globals).
   Globals are necessary for providing a way for the interrupt service routine
   to access the module's internal state.

 - The main point is that you should never try to create more than 1 object
   of the SimpleTimer0 class.

   //TODO: Consider adding some compile-time safety mechanism for verifying
   //      that there is only one object of the SimpleTimer0 class.  This is
   //      simple to do at run-time, but I'm not sure how to do it at
   //      compile-time.

   //TODO: Decide whether or not it's necessary to set the waveform generation
   //      mode and compare output mode inside the constructor.

*******************************************************************************/
#include "SimpleTimer0.hpp"
#include "timer.hpp"
#include <avr/interrupt.h>

static const uint32_t MIN_DURATION   = 1U;     // [milliseconds]
static const uint32_t MAX_DURATION   = 68451U; // [milliseconds]
static const uint32_t CLOCK_RATE_MHz = 16000U; // [MHz]
static const uint32_t MAX_TCNT       = 0x00FFU;

/*------------------------------ Static Globals ------------------------------*/

// Number of times cycled through TCNT reg since SimpleTimer0::start was called
static volatile uint32_t count = 0; 

// Total number of times to cycle through TCNT before stopping
static uint32_t max_count = 0;

/*--------------------- Helper Functions (defined below) ---------------------*/

void reset ();
void set_max_count (uint32_t);

/*------------------------ Interrupt Service Routine -------------------------*/

ISR (TIMER0_OVF_vect)
{
  ++count;
  if (count > max_count)
    reset();
}

/*----------------------- SimpleTimer0 implementation ------------------------*/

SimpleTimer0::SimpleTimer0 (uint16_t length_ms)
{
  set_max_count(length_ms);
  timer0_setWaveformGenerationMode(NORMAL_MAX);
  timer0_setCompareOutputModeA(COMPARE_OUTPUT_ZERO);
  reset();
}

void SimpleTimer0::start ()
{
  timer0_enableTimerOverflowInterrupt();
  timer0_setClockSource(INTERNAL_NO_PRESCALE); // starts the timer module
}

void SimpleTimer0::restart ()
{
  reset();
  start();
}

void SimpleTimer0::set_duration (uint16_t length_ms)
{
	set_max_count(length_ms);
	reset();
}

bool SimpleTimer0::is_finished ()
{
  return timer0_getClockSource() == NO_CLOCK_SOURCE;
}

/*--------------------- Definitions of Helper Functions ----------------------*/

void reset ()
{
  count = 0;
  timer0_disableTimerOverflowInterrupt();
  timer0_setClockSource(NO_CLOCK_SOURCE); // stops the timer module
}

void set_max_count (uint32_t length_ms)
{
  if (length_ms < MIN_DURATION)
    length_ms = MIN_DURATION;
  else if (length_ms > MAX_DURATION)
    length_ms = MAX_DURATION;

  // tolerate following inefficiencies b/c not called often
  max_count = length_ms * CLOCK_RATE_MHz / MAX_TCNT;
}

