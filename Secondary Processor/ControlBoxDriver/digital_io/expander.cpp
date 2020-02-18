#include "expander.hpp"

void Expander::set_address (uint8_t address)
{
  this->address = address;
  i2c.set_address(address);
}

void Expander::write_register (uint8_t reg_addr, uint8_t value)
{
  i2c.set_address(address);
  uint8_t msg [] = {reg_addr, value};
  i2c.write(msg, 2);
}

uint8_t Expander::read_register (uint8_t reg_addr)
{
  i2c.set_address(address);
  i2c.write(&reg_addr, 1);
  uint8_t value;
  i2c.read(&value, 1);
  return value;
}