#ifndef MESSAGE_HPP
#define MESSAGE_HPP

#include <stdint-gcc.h>

struct Message
{
  // constants
  static const uint8_t COUNT_INDEX = 4;
  static const uint8_t DATA_INDEX = 5;
  static const uint8_t LENGTH = 3;
  
  // members
  uint8_t address;
  uint8_t first;
  uint8_t second;
};

#endif // MESSAGE_HPP