#ifndef TIMER_0_H
#define TIMER_0_H

#include <inttypes.h>

  // What is true after a timer has been constructed?
  //   - Set the output compare register
  //   - An appropriate pre-scale value has been selected and this value has
  //     been used to configure the settings register.
  //   - The count variable has been set to an appropriate value so that the
  //     given length is achieved.

  // I must discover the equation governing this system.  The variables
  // involved are the count, the pre-scale setting, the processor's clock rate,
  // the desired timer duration, and the setting of the output compare register.

  /*
       Variable                   Set of Possible Values
  ------------------------------------------------------------------------------
       maxcount                 :  int
       prescale                 :  { 8, 64, 256, 1012 }
       clockrate                :  16,000,000 Hz
       duration                 :  MIN - MAX
       output_compare_register  :  0x00 - 0xFF

  For a timer, I think that we should leave the output compare register at the
  maximum value of that setting (which is 0xFF).  I can achieve the given
  duration by adjusting the maxcount.  Prescale must be selected as well.

  
  */

/*------------------------------- Clock Source -------------------------------*/

typedef enum {
  NO_CLOCK_SOURCE,
  INTERNAL_NO_PRESCALE,
  INTERNAL_PRESCALE_8,
  INTERNAL_PRESCALE_64,
  INTERNAL_PRESCALE_256,
  INTERNAL_PRESCALE_1012,
  EXTERNAL_T0_FALLING_EDGE,
  EXTERNAL_T0_RISING_EDGE
} CLOCK_SOURCE;

void
timer0_setClockSource (CLOCK_SOURCE);

CLOCK_SOURCE
timer0_getClockSource ();

/*------------------------- Waveform Generation Mode -------------------------*/

typedef enum {
  NORMAL_MAX,
  CTC_TOP,
  PWM_PHASE_CORRECT_MAX,
  PWM_PHASE_CORRECT_TOP,
  FAST_PWM_MAX,
  FAST_PWM_TOP,
} WAVEFORM_GENERATION;

void
timer0_setWaveformGenerationMode (WAVEFORM_GENERATION);

WAVEFORM_GENERATION
timer0_getWaveformGenerationMode ();

/*--------------------------- Compare Output mode ----------------------------*/

// See the tables in Section 16 of datasheet for explanation.  The effect of
// these modes will vary according to the other mode settings.
typedef enum {
  COMPARE_OUTPUT_ZERO,
  COMPARE_OUTPUT_ONE,
  COMPARE_OUTPUT_TWO,
  COMPARE_OUTPUT_THREE
} COMPARE_OUTPUT_MODE;

void
timer0_setCompareOutputModeA (COMPARE_OUTPUT_MODE);

COMPARE_OUTPUT_MODE
timer0_getCompareOutputModeA ();

void
timer0_setCompareOutputModeB (COMPARE_OUTPUT_MODE);

COMPARE_OUTPUT_MODE
timer0_getCompareOutputModeB ();

/*------------------------- Output Compare Registers -------------------------*/

void
timer0_setOutputCompareRegisterA (uint8_t);

uint8_t
timer0_getOutputCompareRegisterA ();

void
timer0_setOutputCompareRegisterB (uint8_t);

uint8_t
timer0_getOutputCompareRegisterB ();

/*------------------------- Interrupt Mask Register --------------------------*/

void timer0_enableOutputCompareBInterrupt  ();
void timer0_disableOutputCompareBInterrupt ();
void timer0_enableOutputCompareAInterrupt  ();
void timer0_disableOutputCompareAInterrupt ();
void timer0_enableTimerOverflowInterrupt   ();
void timer0_disableTimerOverflowInterrupt  ();

/*------------------------- High Level Configuration -------------------------*/

void
timer0_configure
(CLOCK_SOURCE s, WAVEFORM_GENERATION w, COMPARE_OUTPUT_MODE c);

#endif // TIMER_0_H
