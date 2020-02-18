#include "digital_pin.hpp"
#include <avr/io.h>
#include <stdint-gcc.h>

/*----------------- Forward Declaration of Helper Functions ------------------*/
/*                      (see below for implementations)                       */

static volatile uint8_t & DDR_REGISTER (Port);
static volatile uint8_t & PORT_REGISTER (Port);
static volatile uint8_t & PIN_REGISTER (Port);
static void configure_as_input (Port, Pin);
static void configure_as_input_pullup (Port, Pin);
static void configure_as_output (Port, Pin);
static void set_pin_mode (Port, Pin, Mode);
static Mode get_pin_mode (Port, Pin);
static void write_to_pin (Port, Pin, bool);
static bool read_from_pin (Port, Pin);

/*---------------------- Public Interface of DigitalPin ----------------------*/

void DigitalPin::initialize (Port port, Pin pin, Mode mode)
{
  this->port = port;
  this->pin = pin;
  this->mode = mode;
  set_pin_mode(port, pin, mode);
}

void DigitalPin::write (bool value)
{
  write_to_pin(port, pin, value);
}

void DigitalPin::toggle ()
{
  write(!read());
}

bool DigitalPin::read ()
{
  return read_from_pin (port, pin);
}

bool DigitalPin::is_high ()
{
  return read();
}

bool DigitalPin::is_low ()
{
  return !read();
}

/*-------------------- Implementation of Helper Functions --------------------*/

static volatile uint8_t error = 0;

static volatile uint8_t & DDR_REGISTER (Port port)
{
  switch (port)
  {
    case A: return DDRA;
    case B: return DDRB;
    case C: return DDRC;
    case D: return DDRD;
    case E: return DDRE;
    case F: return DDRF;
    case G: return DDRG;
    case H: return DDRH;
    case J: return DDRJ;
    case K: return DDRK;
    case L: return DDRL;
  }
  return error;
}

static volatile uint8_t & PORT_REGISTER (Port port)
{
  switch (port)
  {
    case A: return PORTA;
    case B: return PORTB;
    case C: return PORTC;
    case D: return PORTD;
    case E: return PORTE;
    case F: return PORTF;
    case G: return PORTG;
    case H: return PORTH;
    case J: return PORTJ;
    case K: return PORTK;
    case L: return PORTL;
  }
  return error; 
}

static volatile uint8_t & PIN_REGISTER (Port port)
{
  switch (port)
  {
    case A: return PINA;
    case B: return PINB;
    case C: return PINC;
    case D: return PIND;
    case E: return PINE;
    case F: return PINF;
    case G: return PING;
    case H: return PINH;
    case J: return PINJ;
    case K: return PINK;
    case L: return PINL;
  }
  return error;  
}

void configure_as_input (Port port, Pin pin)
{
  DDR_REGISTER(port) &= ~(1 << pin);
  PORT_REGISTER(port) &= ~(1 << pin);
}

void configure_as_input_pullup (Port port, Pin pin)
{
  DDR_REGISTER(port) &= ~(1 << pin);
  PORT_REGISTER(port) |= (1 << pin);
}

void configure_as_output (Port port, Pin pin)
{
  DDR_REGISTER(port) |= (1 << pin);
}

void set_pin_mode (Port port, Pin pin, Mode mode)
{
  if (mode == INPUT)
    configure_as_input(port, pin);
  else if (mode == INPUT_PULLUP)
    configure_as_input_pullup(port, pin);
  else
    configure_as_output(port, pin);
}

Mode get_pin_mode (Port port, Pin pin)
{
  uint8_t direction = DDR_REGISTER(port) & (1 << pin);
  uint8_t port_value = PORT_REGISTER(port) & (1 << pin);
  if (direction)
    return OUTPUT;
  else if (port_value)
    return INPUT_PULLUP;
  else
    return INPUT;
}

void write_to_pin (Port port, Pin pin, bool value)
{
  if (value)
    PORT_REGISTER(port) |= (1 << pin);
  else
    PORT_REGISTER(port) &= ~(1 << pin);
}

bool read_from_pin (Port port, Pin pin)
{
  return PIN_REGISTER(port) & (1 << pin);
}
