#include "touch_screen.hpp"
#include "../memory/memory_variables.hpp"
#include "screen_objects.hpp"
#include "../uart/uart_0.hpp"
#include "../uart/uart_1.hpp"
#include "../uart/uart_2.hpp"
#include "../uart/uart_3.hpp"
#include "../timer/Clock.hpp"
#include <util/delay.h>
#define SCREEN_BAUD 9600
#define CLOCK_RATE 16000000UL

#define when break;case
#define otherwise break;default

ScreenObject_t Form0Objects[FORM_0_C_count];
ScreenObject_t Form1Objects[FORM_1_C_count];
ScreenObject_t Form2Objects[FORM_2_C_count];
ScreenObject_t Form3Objects[FORM_3_C_count];
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
// Form 3

	// Form 0
	Form0Objects[0] = {LEDDigits, 0, 0};
	// Form 1
	Form1Objects[0] = {LEDDigits, Main_Motor_Cell_1, 0};
	Form1Objects[1] = {LEDDigits, Main_Motor_Cell_2, 0};
	Form1Objects[2] = {LEDDigits, Main_Motor_Cell_3, 0};
	Form1Objects[3] = {LEDDigits, Main_Motor_Cell_4, 0};
	Form1Objects[4] = {LEDDigits, Main_Motor_Cell_5, 0};
	Form1Objects[5] = {LEDDigits, Main_Motor_Cell_6, 0};
	Form1Objects[6] = {LEDDigits, Main_Motor_Cell_7, 0};
	Form1Objects[7] = {LEDDigits, Main_Motor_Cell_8, 0};
	Form1Objects[8] = {LEDDigits, Main_EE_Cell_1, 0};
	Form1Objects[9] = {LEDDigits, Main_EE_Cell_2, 0};
	Form1Objects[10] = {LEDDigits, Second_Motor_Cell_1, 0};
	Form1Objects[11] = {LEDDigits, Second_Motor_Cell_2, 0};
	Form1Objects[12] = {LEDDigits, Second_Motor_Cell_3, 0};
	Form1Objects[13] = {LEDDigits, Second_Motor_Cell_4, 0};
	Form1Objects[14] = {LEDDigits, Second_Motor_Cell_5, 0};
	Form1Objects[15] = {LEDDigits, Second_Motor_Cell_6, 0};
	Form1Objects[16] = {LEDDigits, Second_Motor_Cell_7, 0};
	Form1Objects[17] = {LEDDigits, Second_Motor_Cell_8, 0};
	Form1Objects[18] = {LEDDigits, Second_EE_Cell_1, 0};
	Form1Objects[19] = {LEDDigits, Second_EE_Cell_2, 0};
	// From 2
	Form2Objects[0] = {Gauge, 0, 0};
	Form2Objects[1] = {Gauge, 1, 0};
	Form2Objects[2] = {Gauge, 2, 0};
	Form2Objects[3] = {Gauge, 3, 0};
	Form2Objects[4] = {Gauge, 4, 0};
	Form2Objects[5] = {Gauge, 5, 0};
	// Form 3
	Form3Objects[0] = {LEDDigits, X_LOC, 0};
	Form3Objects[1] = {LEDDigits, Y_LOC, 0};
	Form3Objects[2] = {LEDDigits, HEADING, 0};
	Form3Objects[3] = {Led, CONNECTED_LED, 0};
	Form3Objects[4] = {Led, MACRO_RUNNING, 0};
	Form1Objects[5] = {LEDDigits, Main_E_bat, 0};
	Form1Objects[6] = {LEDDigits, Dig_E_bat, 0};
	Form1Objects[7] = {LEDDigits, Main_Motor_Bat, 0};
	Form1Objects[8] = {LEDDigits, Dig_Motor_Bat, 0};


}
void TouchScreen::UpdateObject_data(uint8_t _form) const
{
	if(_form == 0)
	{
		Form0Objects[0].Data = memory->read(CONNECTED_TIME_ELAP);
	}
	else if(_form == 1)
	{
		//Form3Objects[0].Data =
		//Form3Objects[1].Data =
		//Form3Objects[2].Data =
		//Form3Objects[3].Data =
		//Form3Objects[4].Data =
		//Form3Objects[5].Data =
		//Form3Objects[6].Data =
		//Form3Objects[7].Data =
		//Form3Objects[8].Data =
		//Form3Objects[9].Data =
		//Form3Objects[10].Data =
		//Form3Objects[11].Data =
		//Form3Objects[12].Data =
		//Form3Objects[13].Data =
		//Form3Objects[14].Data =
		//Form3Objects[15].Data =
		//Form3Objects[16].Data =
		//Form3Objects[17].Data =
		//Form3Objects[18].Data =
		//Form3Objects[19].Data =
	}
	else if(_form == 2)
	{
		Form2Objects[0].Data = memory->read(JOYSTICK_LEFT_X);
		Form2Objects[1].Data = memory->read(JOYSTICK_LEFT_Y);
		Form2Objects[2].Data = memory->read(SLIDER_LEFT);
		Form2Objects[3].Data = memory->read(JOYSTICK_RIGHT_X);
		Form2Objects[4].Data = memory->read(JOYSTICK_RIGHT_Y);
		Form2Objects[5].Data = memory->read(SLIDER_RIGHT);
	}
	else if(_form == 3)
	{
		Form3Objects[0].Data = memory->read(X_coord)/10;
		Form3Objects[1].Data = memory->read(Y_coord)/10;
		Form3Objects[2].Data = memory->read(HeadingVal);
		Form3Objects[3].Data = (memory->read(CONNECTED_TIME_ELAP) >0? 1:0);
		Form3Objects[4].Data = (memory->read(MACRO_TYPE) > 0? 1:0);
		//Form3Objects[5].Data =
		//Form3Objects[6].Data =
		//Form3Objects[7].Data =
		//Form3Objects[8].Data =
	}

}

/*----------------------------- Updating Screen ------------------------------*/
Clocks screenRefreshTimer(500);
void TouchScreen::refresh() const
{
	if(screenRefreshTimer.isDone()) {
		uint8_t _form = quary_object(Form);
		printf( "Form: %d\n",_form);
		UpdateObject_data(_form);
		ScreenObject_t *ptr;
		switch(_form)
		{
		case 0:
			ptr = Form0Objects;
			break;
		case 1:
			ptr = Form1Objects;
			break;
		case 2:
			ptr = Form2Objects;
			break;
		case 3:
			ptr = Form3Objects;
			break;
		}
		for (uint16_t i = 0; i < FormControlCount[_form]; ++i)
			update_object(ptr[i]);
		screenRefreshTimer.reset();
	}
}

void TouchScreen::update_object (ScreenObject_t i) const
{
	ScreenMessage msg;
	msg.command      = 0x01;
	msg.object_type  = i.Control;
	msg.object_index = i.ID;
	msg.value_msb    = i.Data >> 8;
	msg.value_lsb    = i.Data;
	msg.checksum     = msg.calculate_correct_checksum();
	if(!send_message(msg)) {
		printf("Failed Sending Screen Message\n");
	}
}
void TouchScreen::update_object (uint16_t object,uint16_t _d) const
{

	ScreenMessage msg;
	msg.command      = 0x01;
	msg.object_type  = object >> 8;
	msg.object_index = object;
	msg.value_msb    = _d >> 8;
	msg.value_lsb    = _d;
	msg.checksum     = msg.calculate_correct_checksum();
	//if(send_message(msg)) {
	//printf("Sending Screen Message: %d\nObject: %d\n",value,object);
	//}
}
int TouchScreen::quary_object(uint8_t object) const
{
	uart3_Buffer_Rx_Clear();
	ScreenRequestMessage msg;
	ScreenMessage msgtx;
	Clocks timeout(1000);

	msg.command      = 0x00;
	msg.object_type  = object;
	msg.object_index = 0;
	msg.checksum     = msg.calculate_correct_checksum();

	send_message(msg);
	while(uart3_ReceiveAvailable() < 6)
	{
		if(timeout.isDone())
			return 0xff;
	}
	printf("available: %d\n",uart3_ReceiveAvailable());
	while(msgtx.command != 0x05) {
		msgtx.command      = read_byte();
	}

	msgtx.object_type  = read_byte();
	msgtx.object_index = read_byte();
	msgtx.value_msb    = read_byte();
	msgtx.value_lsb    = read_byte();
	msgtx.checksum     = read_byte();
	printf("Type: %d\nIndex: %d\n",msgtx.object_type,(msgtx.value_msb << 8) | msgtx.value_lsb);
	if(msgtx.command == 0x05 && msgtx.object_type == object) {
		return (msgtx.value_msb << 8) | msgtx.value_lsb;
	}

	return 0xff;




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
bool TouchScreen::send_message (const ScreenRequestMessage & msg) const
{
	static bool good_screen_coms = true;
	const int MAX_ATTEMPTS = 10;
	if (good_screen_coms)
	{
		printf("requesting!\n");
		send_byte(msg.command);
		send_byte(msg.object_type);
		send_byte(msg.object_index);
		send_byte(msg.checksum);
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
