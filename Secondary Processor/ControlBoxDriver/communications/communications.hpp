#ifndef COMMUNICATIONS_HPP
#define COMMUNICATIONS_HPP

#include <stdint-gcc.h>
#include "message.hpp"
#include "message_queue.hpp"
#include "parser.hpp"
#include "../memory/memory.hpp"
#include "../uart/uart_module.hpp"
#include "request.hpp"
#include "../timer/Clock.hpp"

// Fast Transfer
class Communications
{
public:
	// Initialization
	void initialize (UART_MODULE, Memory *);

	// Listening to the Robot
	void receive ();
	void read ();
	uint8_t get_num_messages () const;
	Message get_next_message ();

	// Talking to the Robot
	bool is_macro_requested ();
	void send_stop_macro ();
	int8_t get_requested_macro ();
	void handle_macro_request (uint8_t);
	void handle_manual_command ();
	void stop_all_motors ();
	void send (const Message [], uint8_t);

	// Construct messages
	void get_arm_motor_command (Message [], uint8_t &);
	void get_bucket_motor_command (Message [], uint8_t &);
	void get_plow_motor_command (Message [], uint8_t &);
	void joystickR_msg (Message [], uint8_t &);
	void joystickL_msg (Message messages [], uint8_t & index);
private:

	Memory * memory;
	MessageQueue unread_messages;
	FastTransferParser parser;
	Clocks timeout;
	void initialize_uart ();
	uint16_t read_byte ();
	void send_byte (uint8_t);
	uint8_t crc (const Message [], uint8_t);
	void parse (uint8_t);




	long mapVal(long x, long in_min, long in_max, long out_min, long out_max);
};

#endif // COMMUNICATIONS_HPP
