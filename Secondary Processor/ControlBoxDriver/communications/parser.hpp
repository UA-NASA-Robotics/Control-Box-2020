#ifndef PARSER_HPP
#define PARSER_HPP
#include <stdint-gcc.h>
#include "message.hpp"

class FastTransferParser
{
private:
  static const uint8_t BUFFER_SIZE = 50;
public: 
  FastTransferParser ();
  void push (uint8_t);
  bool valid_messages () const;
  bool is_empty () const;
  bool is_full () const;
  void clear ();
  Message get_message (uint8_t) const;
  uint8_t num_messages () const;
private:
  uint8_t size () const;
  bool verify_crc () const;
  uint8_t buffer [BUFFER_SIZE];
  uint8_t index;
};

#endif // PARSER_HPP