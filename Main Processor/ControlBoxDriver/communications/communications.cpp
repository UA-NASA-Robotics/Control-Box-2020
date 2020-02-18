#include "communications.hpp"
#include "../uart/uart_0.hpp"
#include "../uart/uart_1.hpp"
#include "../uart/uart_2.hpp"
#include "../uart/uart_3.hpp"
#include "robot_addresses.hpp"
#include "../memory/memory_variables.hpp"
#include "../timer/Clock.hpp"
#include "../digital_io/digital_pin.hpp"
#include <stdlib.h>
#include <avr/io.h>
#include <stdio.h>

#define LANTRONIX_BAUD 115200UL
#define CLOCK_RATE 16000000UL
#define UART_ERROR(x) ((x) & 0xFF00)

#define DEADZONE_HIGH 525
#define DEADZONE_LOW  497

const uint8_t FIRST_BYTE = 0x06;
const uint8_t SECOND_BYTE = 0x85;
const uint8_t ROBOT_ADDRESS = 0x04;
const uint8_t CONTROL_BOX_ADDRESS = 0x01;

#define when break;case
#define otherwise break;default

const UART_MODULE uart = UART_1;

Clocks MACRO_RE_SEND_TIMER, ROBOT_PING_TIMER;
Clocks ROBOT_CONNECTED_TIMEOUT, CONNECTED_TIME;
void Communications::initialize (UART_MODULE uart, Memory * memory)
{
	//uart = UART_1;
	this->memory = memory;
	initialize_uart();
	MACRO_RE_SEND_TIMER.setInterval(100);
	ROBOT_PING_TIMER.setInterval(300);
	ROBOT_CONNECTED_TIMEOUT.setInterval(2000);
	CONNECTED_TIME.setInterval(1010);

}

// Listening to the Robot

void Communications::receive ()
{
	read();
	while (get_num_messages() > 0)
	{

		Message msg = get_next_message();
		uint16_t data = (uint16_t)msg.first + ((uint16_t)msg.second << 8);
		if (memory->valid_address(msg.address)) {
			if(msg.address == MACRO_TYPE && data == 0 ) {
				memory->write(memory->read(MACRO_TYPE) -1 + PUSH_BUTTON_0_FLAG, 0);
			}
			memory->write(msg.address, data);
		}
		//if We get a stop macro clear the buttons

	}

}

void Communications::read ()
{
	uint16_t byte = read_byte();
	while (!UART_ERROR(byte))
	{
		parse(byte);
		byte = read_byte();
	}
}

uint8_t Communications::get_num_messages () const
{
	return unread_messages.count();
}

Message Communications::get_next_message ()
{
	return unread_messages.pop();
}

void Communications::check_connection ()
{
	memory->write(CONNECTED, memory->read(MESSAGE_SENDER));
	memory->write(MESSAGE_SENDER, 0);

	if(memory->read(CONNECTED))
	{
		if(CONNECTED_TIME.isDoneSpecial())
			memory->write(CONNECTED_TIME_ELAP, memory->read(CONNECTED_TIME_ELAP) + 1);
		ROBOT_CONNECTED_TIMEOUT.reset();
	}
	if(ROBOT_CONNECTED_TIMEOUT.isDone_NoReset() ) {
		CONNECTED_TIME.reset();
		memory->write(CONNECTED_TIME_ELAP,0);
	}
}

// Talking to the Robot
static Clocks TransmitPeriodTimer(100);
bool E_StopPressed = false;
void Communications::transmit ()
{
	receive();
	ping_robot();
	if (is_emergency_stop_pressed() || E_StopPressed)
	{

		handle_emergency_stop();
		if(memory->read(MACRO_TYPE) != 0) {
			E_StopPressed = true;
			return;
		} else {
			for (uint8_t i = 0; i < NUM_PUSH_BUTTONS; ++i)
			{
				memory->write(PUSH_BUTTON_0_FLAG + i, 0);
			}
			E_StopPressed = false;
		}

	}

	if (is_macro_in_progress())
	{

		return; // Don't do anything.
	}
	else
	{
		if(TransmitPeriodTimer.isDone())
		{
			// send_stop_macro(); // Just in case, write 0 to macro.
			handle_manual_command();
		}
	}
	int requestedMacro = get_requested_macro();
	if (requestedMacro != -1 && requestedMacro != is_macro_in_progress())
	{
		//Trying to Send Macro: get_requested_macro();
		if(MACRO_RE_SEND_TIMER.isDone()) {

			handle_macro_request(get_requested_macro());
			//SendingMacro
		}
	}

}

bool Communications::is_emergency_stop_pressed ()
{
	return memory->read(ARCADE_BUTTON_0_FLAG);
}

int Communications::is_macro_in_progress ()
{
	return memory->read(MACRO_TYPE);
}

bool Communications::is_macro_requested ()
{
	for (uint8_t i = 0; i < NUM_PUSH_BUTTONS; ++i)
		if (memory->read(PUSH_BUTTON_0_FLAG + i))
			return true;
	return false;
}

void Communications::handle_emergency_stop ()
{
	Message message [7];
	message[0] = {LEFT_MOTOR_SPEED, 0, 0};
	message[1] = {RIGHT_MOTOR_SPEED, 0, 0};
	message[2] = {ARM_MOTOR_SPEED, 0, 0};
	message[3] = {BUCKET_ACTUATOR_SPEED, 0, 0};
	message[4] = {PLOW_SPEED_DIRECTION, 0, 0};
	message[5] = {MACRO_COMMAND, 0, 0};
	message[6] = {MACRO_ARGUMENT, 0, 0};

	//do
	//{
	send(message, 7);
	receive();
	//} while (memory->read(MACRO_TYPE) != 0 && !timeout.isDone());
//  if(memory->read(MACRO_TYPE) != 0)
//    printf("TimeOut\r\n");
}

void Communications::send_stop_macro ()
{
	Message message [2];
	message[0] = {MACRO_COMMAND, 0, 0};
	message[1] = {MACRO_ARGUMENT, 0, 0};
	timeout.reset();
	do
	{
		send(message, 2);
		receive();
	} while (memory->read(MACRO_TYPE) != 0 && !timeout.isDone());    //This is to ensure the that robot is out of the macro
}

void Communications::ping_robot ()
{
	if(ROBOT_PING_TIMER.isDone())
	{
		//ping
		Message message [0];
		message[0] = {MESSAGE_SENDER, CONTROL_BOX_ADDRESS, 0};
		send(message, 1);
	}

}

int8_t Communications::get_requested_macro ()
{
	uint8_t macro = -1;
	for (uint8_t i = 0; i < NUM_PUSH_BUTTONS; ++i) {
		if (memory->read(PUSH_BUTTON_0_FLAG + i))
			return i+1;//macro = i + 1;
	}
	return macro;
}

void Communications::handle_macro_request (uint8_t macro)
{
	Message message [2];
	message[0] = {MACRO_COMMAND, macro, 0};
	message[1] = {MACRO_ARGUMENT, 0, 0};
	timeout.reset();
	//do
	//{
	send(message, 2);
	receive();
	//} while (memory->read(MACRO_TYPE) != macro && !timeout.isDone());
}

void Communications::handle_manual_command ()
{
	Message messages [5];
	uint8_t index = 0;
	joystickR_msg(messages, index);
	joystickL_msg(messages, index);
	//get_arm_motor_command(messages, index);
	get_bucket_motor_command(messages, index);
	get_plow_motor_command(messages, index);
	send(messages, 5);
}

void Communications::stop_all_motors ()
{
	Message messages [5] =
	{
		Message {LEFT_MOTOR_SPEED, 0, 0},
		Message {RIGHT_MOTOR_SPEED, 0, 0},
		Message {ARM_MOTOR_SPEED, 0, 0},
		Message {BUCKET_ACTUATOR_SPEED, 0, 0},
		Message {PLOW_SPEED_DIRECTION, 0, 0}
	};
	send(messages, 5);
}

void Communications::get_arm_motor_command (Message messages [], uint8_t & index)
{
	Message message {ARM_MOTOR_SPEED, 0, 0};

	if (memory->read(ARCADE_BUTTON_3_FLAG))
	{
		message.first = 50;
		message.second = 0;
	}
	else if (memory->read(ARCADE_BUTTON_6_FLAG))
	{
		message.first = ~50 + 1;
		message.second = 0xFF;
	}
	messages[index++] = message;
}

void Communications::get_bucket_motor_command (Message messages [], uint8_t & index)
{
	Message message {BUCKET_ACTUATOR_SPEED, 0, 0};
	int slider = memory->read(SLIDER_RIGHT);
	int msgVal = mapVal(slider, 0,1024, 15,100);
	if (memory->read(ARCADE_BUTTON_5_FLAG))
	{
		msgVal = msgVal*(-1);
		message.first = msgVal & 0xFF;
		message.second = (msgVal >> 8) & 0xFF;
	}
	else if (memory->read(ARCADE_BUTTON_4_FLAG))
	{
		message.first = msgVal & 0xFF;
		message.second = (msgVal >> 8) & 0xFF;
	}
	messages[index++] = message;
}

void Communications::get_plow_motor_command (Message messages [], uint8_t & index)
{
	Message message {PLOW_SPEED_DIRECTION, 0, 0};
	if (memory->read(ARCADE_BUTTON_1_FLAG) || memory->read(ARCADE_BUTTON_7_FLAG))
	{
		message.first = ~100 + 1;
		message.second = 0xFF;
	}
	else if (memory->read(ARCADE_BUTTON_2_FLAG) || memory->read(ARCADE_BUTTON_8_FLAG))
	{
		message.first = 100;
		message.second = 0x00;
	}
	messages[index++] = message;
}


void Communications::joystickR_msg (Message messages [], uint8_t & index)
{
	static const double vertical_compensation = 1;
	static const double horizontal_compensation = 0.5;
	static const int max_speed = 100;
	static const int min_speed = -100;
	int X = memory->read(JOYSTICK_LEFT_X) / 5 - 100;
	int Y = memory->read(JOYSTICK_LEFT_Y) / 5 - 100;
	int slider = memory->read(SLIDER_RIGHT);
	int leftSpeed, rightSpeed;

	slider = mapVal(slider, 0,1000, 15,100);

	leftSpeed = (int)(Y * vertical_compensation + X * horizontal_compensation);
	rightSpeed = (int)(Y * vertical_compensation - X * horizontal_compensation);

	if (leftSpeed > max_speed)
		leftSpeed = max_speed;
	else if (leftSpeed < min_speed)
		leftSpeed = min_speed;
	if (rightSpeed > max_speed)
		rightSpeed = max_speed;
	else if (rightSpeed < min_speed)
		rightSpeed = min_speed;

	leftSpeed  = mapVal(leftSpeed, min_speed, max_speed, -slider,slider);
	rightSpeed = mapVal(rightSpeed, min_speed, max_speed, -slider,slider);

	/* Converting to two's Comp */
	uint16_t left_speed = leftSpeed < 0 ? ~(uint16_t)(abs(leftSpeed)) + 1 : (uint16_t)leftSpeed;
	uint16_t right_speed = rightSpeed < 0 ? ~(uint16_t)(abs(rightSpeed)) + 1 : (uint16_t)rightSpeed;
//  if(abs(right_speed) > 15 || abs(left_speed) > 15)
//  {
	//printf("RightSpeed: %d\nLeftSpeed: %d\n",right_speed,left_speed);
//  }
//  printf("RightSpeed: %d\nLeftSpeed: %d\n",right_speed,left_speed);

	messages[index].address = LEFT_MOTOR_SPEED;
	messages[index].first = left_speed;
	messages[index].second = left_speed >> 8;
	++index;
	messages[index].address = RIGHT_MOTOR_SPEED;
	messages[index].first = right_speed;
	messages[index].second = right_speed >> 8;
	++index;
}
void Communications::joystickL_msg (Message messages [], uint8_t & index)
{
	int max_speed = 100;
	int min_speed = -100;
	int upSpeed = memory->read(JOYSTICK_RIGHT_Y) / 5 - 100;
	int slider = memory->read(SLIDER_LEFT);


	slider = mapVal(slider, 0,1000, 15,100);


	if (upSpeed > max_speed)
		upSpeed = max_speed;
	else if (upSpeed < min_speed)
		upSpeed = min_speed;


	upSpeed  = mapVal(upSpeed, min_speed, max_speed, -slider,slider);

	//printf("Arm: %d\r",upSpeed);

	messages[index].address = ARM_MOTOR_SPEED;
	messages[index].first = upSpeed;
	messages[index].second = upSpeed >> 8;
	++index;
}
void Communications::send (const Message messages [], uint8_t count)
{
	uint8_t cyclic_redundancy_check = crc(messages, count);
	send_byte(FIRST_BYTE);
	send_byte(SECOND_BYTE);
	send_byte(ROBOT_ADDRESS);
	send_byte(CONTROL_BOX_ADDRESS);
	send_byte(Message::LENGTH * count);
	for (int i = 0; i < count; ++i)
	{
		send_byte(messages[i].address);
		send_byte(messages[i].first);
		send_byte(messages[i].second);
	}
	send_byte(cyclic_redundancy_check);
}

void Communications::initialize_uart ()
{
	switch (uart)
	{
	case UART_0:
		uart0_initialize(UART0_BAUD_SELECT(LANTRONIX_BAUD, CLOCK_RATE));
when UART_1:
		uart1_initialize(UART1_BAUD_SELECT(LANTRONIX_BAUD, CLOCK_RATE));
when UART_2:
		uart2_initialize(UART2_BAUD_SELECT(LANTRONIX_BAUD, CLOCK_RATE));
when UART_3:
		uart3_initialize(UART3_BAUD_SELECT(LANTRONIX_BAUD, CLOCK_RATE));
	}
}

uint16_t Communications::read_byte ()
{
	switch (uart)
	{
	case UART_0:
		return uart0_read_byte();
		break;
	case UART_1:
		return uart1_read_byte();
		break;
	case UART_2:
		return uart2_read_byte();
		break;
	case UART_3:
		return uart3_read_byte();
		break;
	}
	return 0xFF00;
}

void Communications::send_byte (uint8_t value)
{
	//uart = UART_1;
	switch (uart)
	{
	case UART_0:
		uart0_send_byte(value);
		break;
	case UART_1:
		uart1_send_byte(value);
		break;
	case UART_2:
		uart2_send_byte(value);
		break;
	case UART_3:
		uart3_send_byte(value);
	}

}

uint8_t Communications::crc (const Message messages [], uint8_t count)
{
	const uint8_t CRC_POLYNOMIAL = 0x8C;
	uint8_t value = 0x00;
	for (uint8_t i = 0; i < count; ++i)
	{
		uint8_t arr [] = {messages[i].address, messages[i].first, messages[i].second};
		for (uint8_t j = 0; j < 3; ++j)
		{
			uint8_t data = arr[j];
			for (uint8_t k = 0; k < 8; ++k)
			{
				uint8_t sum = (value ^ data) & 0x01;
				value >>= 1;
				if (sum)
					value ^= CRC_POLYNOMIAL;
				data >>= 1;
			}
		}
	}
	return value;
}

void Communications::parse (uint8_t byte)
{
	parser.push(byte);
	if (parser.valid_messages())
	{
		//printf("We got a valid Message!\r\n");
		//Getting the number of message indexes have been received in the packet
		uint8_t max = parser.num_messages();
		for (uint8_t i = 0; i < max; ++i)
			unread_messages.push(parser.get_message(i));
		parser.clear();
	}
	else if (parser.is_full())
		parser.clear();
}

long Communications::mapVal(long x, long in_min, long in_max, long out_min, long out_max)
{
	return  (x - in_min) * (out_max - out_min ) / (in_max - in_min) + out_min;
}