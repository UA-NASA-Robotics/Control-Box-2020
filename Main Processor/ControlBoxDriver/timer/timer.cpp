#include "timer.hpp"
#include <avr/io.h>

static CLOCK_SOURCE cs;
static WAVEFORM_GENERATION wgm;
static COMPARE_OUTPUT_MODE comA;
static COMPARE_OUTPUT_MODE comB;

/*------------------------------- Clock Source -------------------------------*/

void
timer0_setClockSource (CLOCK_SOURCE mode)
{
	cs = mode;

	// Clear the bits.
	TCCR0B &= 0b11111000;

	switch (mode)
	{
		case NO_CLOCK_SOURCE:
			TCCR0B |= 0b00000000;
			break;
		case INTERNAL_NO_PRESCALE:
			TCCR0B |= 0b00000001;
			break;
		case INTERNAL_PRESCALE_8:
			TCCR0B |= 0b00000010;
			break;
		case INTERNAL_PRESCALE_64:
			TCCR0B |= (1 << CS01) | (1 << CS00);
			//TCCR0B |= 0b00000011;
			break;
		case INTERNAL_PRESCALE_256:
			TCCR0B |= 0b00000100;
			break;
		case INTERNAL_PRESCALE_1012:
			TCCR0B |= 0b00000101;
			break;
		case EXTERNAL_T0_FALLING_EDGE:
			TCCR0B |= 0b00000110;
			break;
		case EXTERNAL_T0_RISING_EDGE:
			TCCR0B |= 0b00000111;
			break;
	}
}

CLOCK_SOURCE
timer0_getClockSource ()
{
	return cs;
}

/*------------------------- Waveform Generation Mode -------------------------*/

void
timer0_setWaveformGenerationMode (WAVEFORM_GENERATION mode)
{
	wgm = mode;

	// Clear the bits.
	TCCR0A &= 0b11111100; // Last two bits are WGM1 and WGM0
	TCCR0B &= 0b11110111; // Bit 3 is WGM2

	switch (mode)
	{
		case NORMAL_MAX:
			break;
		case CTC_TOP:
			TCCR0A |= 0b00000010;
			break;
		case PWM_PHASE_CORRECT_MAX:
			TCCR0A |= 0b00000001;
			break;
		case PWM_PHASE_CORRECT_TOP:
			TCCR0A |= 0b00000001;
			TCCR0B |= 0b00001000;
			break;
		case FAST_PWM_MAX:
			TCCR0A |= 0b00000011;
			break;
		case FAST_PWM_TOP:
			TCCR0A |= 0b00000011;
			TCCR0B |= 0b00001000;
			break;
	}
}

WAVEFORM_GENERATION
timer0_getWaveformGenerationMode ()
{
	return wgm;
}

/*--------------------------- Compare Output Mode ----------------------------*/

void
timer0_setCompareOutputModeA (COMPARE_OUTPUT_MODE mode)
{
	TCCR0A &= 0b00111111; // clear bits 7 and 6
	comA = mode;
	switch (mode)
	{
		case COMPARE_OUTPUT_ZERO:
			// leave bits 7 and 6 cleared
			break;
		case COMPARE_OUTPUT_ONE:
			TCCR0A |= 0b01000000;
			break;
		case COMPARE_OUTPUT_TWO:
			TCCR0A |= 0b10000000;
			break;
		case COMPARE_OUTPUT_THREE:
			TCCR0A |= 0b11000000;
			break;
	}
}

COMPARE_OUTPUT_MODE
timer0_getCompareOutputModeA ()
{
	return comA;
}

void
timer0_setCompareOutputModeB (COMPARE_OUTPUT_MODE mode)
{
	TCCR0A &= 0b11001111; // clear bits 5 and 4
	comA = mode;
	switch (mode)
	{
		case COMPARE_OUTPUT_ZERO:
			// leave bits 5 and 4 cleared
			break;
		case COMPARE_OUTPUT_ONE:
			TCCR0A |= 0b00010000;
			break;
		case COMPARE_OUTPUT_TWO:
			TCCR0A |= 0b00100000;
			break;
		case COMPARE_OUTPUT_THREE:
			TCCR0A |= 0b00110000;
			break;
	}
}

COMPARE_OUTPUT_MODE
timer0_getCompareOutputModeB ()
{
	return comB;
}

/*------------------------- Output Compare Registers -------------------------*/

void
timer0_setOutputCompareRegisterA (uint8_t top)
{
	OCR0A = top;
}

uint8_t
timer0_getOutputCompareRegisterA ()
{
	return OCR0A;
}

void
timer0_setOutputCompareRegisterB (uint8_t top)
{
	OCR0B = top;
}

uint8_t
timer0_getOutputCompareRegisterB ()
{
	return OCR0B;
}

/*------------------------- Interrupt Mask Register --------------------------*/

void
timer0_enableOutputCompareBInterrupt ()
{
	// Write HIGH to TIMSK0 (Bit 2)
	TIMSK0 |= 0b00000100;
}

void
timer0_disableOutputCompareBInterrupt ()
{
	// Write LOW to TIMSK0 (Bit 2)
	TIMSK0 &= 0b11111011;
}

void
timer0_enableOutputCompareAInterrupt ()
{
	// Write HIGH to TIMSK0 (Bit 1)
	TIMSK0 |= 0b00000010;
}

void timer0_disableOutputCompareAInterrupt ()
{
	// Write LOW to TIMSK0 (Bit 1)
	TIMSK0 &= 0b11111101;
}

void
timer0_enableTimerOverflowInterrupt ()
{
	// Write HIGH to TIMSK0 (Bit 0)
	TIMSK0 |= 0b00000001;
}

void
timer0_disableTimerOverflowInterrupt ()
{
	// Write LOW to TIMSK0 (Bit 0)
	TIMSK0 &= 0b11111110;
}

/*------------------------- High Level Configuration -------------------------*/

void
timer0_configure
(CLOCK_SOURCE s, WAVEFORM_GENERATION w, COMPARE_OUTPUT_MODE c)
{
	timer0_setClockSource(s);
	timer0_setWaveformGenerationMode(w);
	timer0_setCompareOutputModeA(c);
}
