#ifndef ANALOG_INPUT_HPP
#define ANALOG_INPUT_HPP

#include <stdint-gcc.h>

typedef enum
{
  CHANNEL_ZERO,
  CHANNEL_ONE,
  CHANNEL_TWO,
  CHANNEL_THREE,
  CHANNEL_FOUR,
  CHANNEL_FIVE,
  CHANNEL_SIX,
  CHANNEL_SEVEN
} ANALOG_CHANNEL;

class AnalogInput
{
public:
  void initialize (ANALOG_CHANNEL);
  uint16_t read ();
private:
  void select_voltage_reference ();
  void switch_to_this_channel ();
  void disable_digital_input_buffers ();
  void set_prescale ();
  void enable_adc ();
  void start_conversion ();
  ANALOG_CHANNEL channel;
};

#endif // ANALOG_INPUT_HPP