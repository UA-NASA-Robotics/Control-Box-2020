#include "analog_input.hpp"
#include <avr/io.h>
#include <avr/interrupt.h>

#define CLEAR_CHANNEL_MASK_1 0b11100000
#define CLEAR_CHANNEL_MASK_2 0b11110111

static volatile bool conversion_complete = false;

ISR(ADC_vect)
{
  conversion_complete = true;
}

void AnalogInput::initialize (ANALOG_CHANNEL channel)
{
  sei();
  this->channel = channel;
  select_voltage_reference();
  disable_digital_input_buffers();
  set_prescale();
  enable_adc();
}

uint16_t AnalogInput::read ()
{
  switch_to_this_channel();
  start_conversion();
  while (!conversion_complete);
  uint16_t low = ADCL;
  uint16_t high = ADCH;
  return low + (high << 8);
}

void AnalogInput::select_voltage_reference ()
{
  ADMUX |= (1 << REFS0); // Select AVCC as voltage reference for ADC
}

void AnalogInput::switch_to_this_channel ()
{
  ADMUX &= CLEAR_CHANNEL_MASK_1;
  ADCSRB &= CLEAR_CHANNEL_MASK_2;
  ADMUX |= channel;
}

void AnalogInput::disable_digital_input_buffers ()
{
  DIDR0 |= 0xFF;
}

void AnalogInput::set_prescale ()
{
  // Divide the clock speed (16MHz) by 128, to provide a frequency of
  // 125kHz to the ADC input clock.  The acceptable range is 50kHz to 200kHz.
  ADCSRA |= 0b00000111;
}

void AnalogInput::enable_adc ()
{
  ADCSRA |= (1 << ADEN);
}

void AnalogInput::start_conversion ()
{
  conversion_complete = false;
  ADCSRA |= (1 << ADIE); // Enable the ADC interrupt.
  ADCSRA |= (1 << ADSC); // Start the conversion.
}

