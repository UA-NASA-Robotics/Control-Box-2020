#include "communications.hpp"
#include "../uart/uart_0.hpp"
#include "../uart/uart_1.hpp"
#include "../uart/uart_2.hpp"
#include "../uart/uart_3.hpp"
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


void Communications::initialize (UART_MODULE uart, Memory * memory)
{
	//uart = UART_1;
	this->memory = memory;
	initialize_uart();
	
}

// Listening to the Robot

void Communications::receive ()
{
	read();
	while (get_num_messages() > 0)
	{
		Message msg = get_next_message();
		uint16_t data = (uint16_t)msg.first + ((uint16_t)msg.second << 8);
		if (memory->valid_address(msg.address))
		{
			if(msg.address == MACRO_TYPE && data == 0 )
			{
				memory->write(memory->read(MACRO_TYPE) -1 + PUSH_BUTTON_0_FLAG, 0);
			} // end inner if
			memory->write(msg.address, data);
		} // end outer if
		// if We get a stop macro clear the buttons
	} // end while
} // end receive()

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


// Talking to the Robot
static Clocks TransmitPeriodTimer(100);
bool E_StopPressed = false;

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
