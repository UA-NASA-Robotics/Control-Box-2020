#ifndef EXPANDER_HPP
#define EXPANDER_HPP
#include "../i2c/i2c.hpp"

class Expander // MCP23017
{
public:
  void set_address (uint8_t);
  void write_register (uint8_t, uint8_t);
  uint8_t read_register (uint8_t);
private:
  uint8_t address;
  I2C i2c;
};

#endif // EXPANDER_HPP
