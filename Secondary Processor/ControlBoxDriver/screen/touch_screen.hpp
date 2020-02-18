#ifndef TOUCH_SCREEN_HPP
#define TOUCH_SCREEN_HPP
#include "../memory/memory.hpp"
#include "../uart/uart_module.hpp"

struct ScreenMessage
{
  uint8_t command;
  uint8_t object_type;
  uint8_t object_index;
  uint8_t value_msb;
  uint8_t value_lsb;
  uint8_t checksum;
  uint8_t calculate_correct_checksum ()
  {
    return command ^ object_type ^ object_index ^ value_msb ^ value_lsb;
  }
};

class TouchScreen
{
public:
  void initialize (Memory *, UART_MODULE);
  void refresh () const;
private:
  void assign_objects ();
  void assign_objects_for_test ();
  void update_object (uint16_t) const;
  void log_string () const;
  void send_byte (uint8_t) const;
  bool send_message (const ScreenMessage &) const;
  uint8_t read_byte () const;
  bool received_ack () const;
private:
  uint16_t page;
  Memory * memory;
  UART_MODULE uart_module;
  uint16_t objects [Memory::SIZE];
};

#endif // TOUCH_SCREEN_HPP
