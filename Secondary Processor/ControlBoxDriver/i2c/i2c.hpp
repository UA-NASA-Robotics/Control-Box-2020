#ifndef I2C_HPP
#define I2C_HPP
#include <stdint-gcc.h>

const uint8_t MAX_BUFFER_SIZE = 10; // max num bytes to be read/written

class I2C
{
public:
  void set_address (uint8_t);
  int write (uint8_t *, uint8_t);
  int read  (uint8_t *, uint8_t);

private:
/*------------------------------ Private Types -------------------------------*/

  typedef enum {READ, WRITE} Direction;
  typedef enum {START, VERIFY_START, SEND_ADDRESS, VERIFY_ADDRESS, SEND_BYTE, 
                READ_BYTE, VERIFY_DATA, STOP, DONE} State;

/*----------------------------- Helper Functions -----------------------------*/

  void reset (uint8_t);
  void start ();
  void verify_start ();
  void send_address (I2C::Direction);
  void verify_write_address_sent ();
  void verify_read_address_sent ();
  void send_data_byte (uint8_t);
  void read_data_byte ();
  void verify_data_sent ();
  void verify_data_read ();
  void stop ();
  void fail ();

/*----------------------------- Member Variables -----------------------------*/

  State state;
  uint8_t address;
  uint8_t buffer [MAX_BUFFER_SIZE];
  uint8_t index;
  uint8_t size;
  uint8_t fail_count;
};

#endif // I2C_HPP
