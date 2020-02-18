#include "touch_screen.hpp"
#include "../memory/memory_variables.hpp"
#include "screen_objects.hpp"
#include "../uart/uart_0.hpp"
#include "../uart/uart_1.hpp"
#include "../uart/uart_2.hpp"
#include "../uart/uart_3.hpp"
#include "../timer/Clock.hpp"

#define SCREEN_BAUD 9600
#define CLOCK_RATE 16000000UL

#define when break;case
#define otherwise break;default

/*------------------------------ Initialization ------------------------------*/

void TouchScreen::initialize (Memory * memory, UART_MODULE uart_module)
{
	this->page = 0x00;
	this->memory = memory;
	this->uart_module = uart_module;
	assign_objects();
	switch (uart_module)
	{
	case UART_0:
		uart0_initialize(UART0_BAUD_SELECT(SCREEN_BAUD, CLOCK_RATE));
when UART_1:
		uart1_initialize(UART1_BAUD_SELECT(SCREEN_BAUD, CLOCK_RATE));
when UART_2:
		uart2_initialize(UART2_BAUD_SELECT(SCREEN_BAUD, CLOCK_RATE));
otherwise:
		uart3_initialize(UART3_BAUD_SELECT(SCREEN_BAUD, CLOCK_RATE));
	}
	printf("Screen UART INIT: %d \n",uart_module);
}

void TouchScreen::assign_objects ()
{
	for (uint16_t i = 0; i < Memory::SIZE; ++i)
		objects[i] = NO_OBJECT;

	// Form 0
	objects[CONNECTED_TIME_ELAP] = LEDDigits;
	//memory->write(CONNECTED_TIME_ELAP,55);

	// Form 1

	// Form 2
//	objects[JOYSTICK_LEFT_X] = Gauge0;
//	objects[JOYSTICK_LEFT_Y] = Gauge1;
//  objects[SLIDER_LEFT] = Gauge2;
//	objects[JOYSTICK_RIGHT_X] = Gauge3;
	//objects[JOYSTICK_RIGHT_Y] = Gauge4;
//  objects[SLIDER_RIGHT] = Gauge5;
}

/*----------------------------- Updating Screen ------------------------------*/
Clocks screenRefreshTimer(500);
void TouchScreen::refresh() const
{
	if(screenRefreshTimer.isDone()) {
		//printf("ScreenUpTime:%d\r\n",memory->read(CONNECTED_TIME_ELAP));
		static int count = 0;
		static const int rate = 10;
		if (count++ >= rate)
		{
			count = 0;
			return;
		}
		for (uint16_t i = 0; i < Memory::SIZE; ++i)
			update_object(i);
		screenRefreshTimer.reset();
	}


}

void TouchScreen::update_object (uint16_t i) const
{

	uint16_t object = objects[i];
	if (object == NO_OBJECT)
		return;
	uint16_t value = memory->read(i);

	ScreenMessage msg;
	msg.command      = 0x01;
	msg.object_type  = object >> 8;
	msg.object_index = object;
	msg.value_msb    = value >> 8;
	msg.value_lsb    = value;
	msg.checksum     = msg.calculate_correct_checksum();
	//if(send_message(msg)) {
	//printf("Sending Screen Message: %d\nObject: %d\n",value,object);
	//}

}

/*-------------------------- Screen Communications ---------------------------*/

void TouchScreen::send_byte (uint8_t value) const
{

	switch (uart_module)
	{
	case UART_0:
		uart0_send_byte(value);
when UART_1:
		uart1_send_byte(value);
when UART_2:
		uart2_send_byte(value);
otherwise: {
			uart3_send_byte(value);
		}
	}
}

uint8_t TouchScreen::read_byte () const
{
	switch (uart_module)
	{
	case UART_0:
		return uart0_read_byte();
	case UART_1:
		return uart1_read_byte();
	case UART_2:
		return uart2_read_byte();
	case UART_3:
		return uart3_read_byte();
	}
	return 0;
}
/*
bool TouchScreen::send_message (const ScreenMessage & msg) const
{
  const int MAX_ATTEMPTS = 10;
  int i = 0;
  bool success = false;
  do
  {
    send_byte(msg.command);
    send_byte(msg.object_type);
    send_byte(msg.object_index);
    send_byte(msg.value_msb);
    send_byte(msg.value_lsb);
    send_byte(msg.checksum);
    success = received_ack();
  } while (!success && (i++ < MAX_ATTEMPTS));
  return success;
}
*/
bool TouchScreen::send_message (const ScreenMessage & msg) const
{
	static bool good_screen_coms = true;
	const int MAX_ATTEMPTS = 10;
	if (good_screen_coms)
	{
		int i = 0;
		bool success = false;
		do
		{
			send_byte(msg.command);
			send_byte(msg.object_type);
			send_byte(msg.object_index);
			send_byte(msg.value_msb);
			send_byte(msg.value_lsb);
			send_byte(msg.checksum);
			success = received_ack();
			++i;
		} while (!success && (i < MAX_ATTEMPTS));
		if (i >= MAX_ATTEMPTS)
			printf("Fail:!!\r\n");
//      good_screen_coms = false;
		return success;
	}
	return false;
}

bool TouchScreen::received_ack () const
{
	const uint16_t NACK = 0x0015;
	const uint16_t ACK = 0x0006;
	Clocks timeout(200); //milliseconds
//  printf("timeout:\n");
//  printf("currentTime: %u\n", timeout.currentTime);
//  printf("startTime: %u\n", timeout.startTime);
//  printf("length: %u\n", timeout.length);
	while (!timeout.isDone())
	{
		uint16_t val = read_byte();
		if (val == NACK) {
			return false;
		}
		if (val == ACK) {
			return true;
		}
	}
	return false;
}
