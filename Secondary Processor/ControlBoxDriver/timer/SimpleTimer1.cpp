#include "SimpleTimer1.hpp"
#include <avr/io.h>
#include <avr/interrupt.h>

static volatile uint32_t count = 0;
static volatile uint32_t GlobalCount = 0;
static uint32_t max_count = 0;

static void reset ()
{
	count = 0;
	TIMSK1 &= 0b11111110; // disable timer overflow interrupt
	TCCR1B &= 0b11111000; // set clock source to no clock source
}

ISR (TIMER1_OVF_vect)
{

	++count;
	if (count > max_count)
	{
		GlobalCount+=4;
		count = 0;
		//reset();
	}
	TIFR1 = (0<<TOV1); //Reset timer0 overflow interrupt flag
}

SimpleTimer1::SimpleTimer1 ()
{
	start();
}
SimpleTimer1::SimpleTimer1(int l)
{
	reset();
	set_duration(l);
	start();

}
uint32_t SimpleTimer1::millis()
{
	return GlobalCount;
}
void SimpleTimer1::start ()
{
	TIMSK1 |= 0b00000001; // enable timer overflow interrupt
	TCCR1B |= 0b00000001; // internal clock source, no prescale

}

void SimpleTimer1::restart ()
{
	reset();
	start();
}

void SimpleTimer1::set_duration (uint32_t length_ms)
{
	const uint32_t CLOCK_RATE_kHz = 16000;
	const uint32_t MAX_TCNT = 0xFFFF;
	max_count = length_ms * CLOCK_RATE_kHz / MAX_TCNT;
}

bool SimpleTimer1::is_finished ()
{
	return !(TCCR1B & 0b00000111);
}

void SimpleTimer1::reset ()
{
	count = 0;
	TIMSK1 &= 0b11111110; // disable timer overflow interrupt
	TCCR1B &= 0b11111000; // set clock source to no clock source
}
