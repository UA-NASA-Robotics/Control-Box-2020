#include "expander_pin.hpp"

// Expander Register Addresses
const uint8_t IODIRA = 0x00;
const uint8_t IODIRB = 0x01;
const uint8_t GPIOA  = 0x12;
const uint8_t GPIOB  = 0x13;

void ExpanderPin::initialize (Expander * expander, Port port, Pin pin, Mode mode)
{
  this->expander = expander;
  this->port = port;
  this->pin = pin;
  set_mode(mode);
}

bool ExpanderPin::read ()
{
  uint8_t reg_addr = (port == A) ? GPIOA : GPIOB;
  return expander->read_register(reg_addr) & (1 << pin);
}

void ExpanderPin::write (bool value)
{
  uint8_t reg_addr = (port == A) ? GPIOA : GPIOB;
  if (value)
    expander->write_register(reg_addr, expander->read_register(reg_addr) | (1 << pin));
  else
    expander->write_register(reg_addr, expander->read_register(reg_addr) & ~(1 << pin));
}

void ExpanderPin::set_mode (Mode mode)
{
  this->mode = mode;
  uint8_t addr = (port == A) ? IODIRA : IODIRB;
  if (mode == INPUT)
    expander->write_register(addr, expander->read_register(addr) | (1 << pin));
  else
    expander->write_register(addr, expander->read_register(addr) & ~(1 << pin));
}

