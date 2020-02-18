#include "output_handler.hpp"
#include "../digital_io/digital_io_params.hpp"
#include "../digital_io/digital_pin.hpp"
#include "../communications/request.hpp"
#include "../memory/memory_variables.hpp"
#include "../timer/Clock.hpp"
#include "../i2c/SevenSegment.hpp"
#include "../i2c/LiquidCrystal_I2C.hpp"
#include <stdio.h>

#define F_CPU 16000000UL
#include <util/delay.h>
Adafruit_7segment LsevenSeg(0x70);
Adafruit_7segment RsevenSeg(0x71);
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7);

void OutputHandler::initialize (Memory * memory)
{
	this->memory = memory;
	initialize_expanders();
	initialize_push_button_leds();
	initialize_panel_leds();

	screen.initialize(memory, UART_3);

	lcd.begin (20, 4); // 20 x 4 LCD module
	lcd.setBacklightPin(3, POSITIVE); // BL, BL_POL
	lcd.setBacklight(1);
	lcd.print("NASA Robotics");
	lcd.setCursor(0, 1);
	lcd.print("Control Box: V2.3");
	LsevenSeg.begin(0x70);
	RsevenSeg.begin(0x71);
}

void OutputHandler::startup ()
{
	// flash the panel-mounted LEDs
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			int index = i % 2 ? 3 - j : j;
			panel_leds[index].write(1);
			panel_leds[index + 4].write(1);
			_delay_ms(50);
			panel_leds[index].write(0);
			panel_leds[index + 4].write(0);
		}
	}

	// flash the push-button LEDs
	spiral();
}

void OutputHandler::refresh ()
{
	refresh_push_button_leds();
	refresh_panel_leds();

	// update the touchscreen
	screen.refresh();

	// update the up-time
	LsevenSeg.print(memory->read(CONNECTED_TIME_ELAP), DEC);
	LsevenSeg.writeDisplay();

	// update the force sensor feedback
	uint16_t force_measurement = memory->read(FORCE_SENSOR_FEEDBACK);
	RsevenSeg.print(force_measurement, DEC);
	RsevenSeg.writeDisplay();
	
	// Row 3 of LCD displays the right slider position when moved
	lcd.setCursor(0, 2);
	char rightarray[50];
	sprintf(rightarray, "Slider: %d", memory->read(SLIDER_RIGHT));
	lcd.print(rightarray);
	
	// Row 4 of LCD displays the left slider position when moved
	lcd.setCursor(0, 3);
	char leftarray[50];
	sprintf(leftarray, "Slider: %d", memory->read(SLIDER_LEFT));
	lcd.print(leftarray);
}

void OutputHandler::initialize_expanders ()
{
	expander_left.set_address(0x20);
	expander_right.set_address(0x27);
}

void OutputHandler::initialize_push_button_leds ()
{
	// Left push-button LEDs
	push_button_leds[0].initialize(&expander_left, A, ONE, OUTPUT);
	push_button_leds[1].initialize(&expander_left, A, THREE, OUTPUT);
	push_button_leds[2].initialize(&expander_left, A, FIVE, OUTPUT);
	push_button_leds[3].initialize(&expander_left, A, SEVEN, OUTPUT);
	push_button_leds[4].initialize(&expander_left, B, ONE, OUTPUT);
	push_button_leds[5].initialize(&expander_left, B, THREE, OUTPUT);
	push_button_leds[6].initialize(&expander_left, B, FIVE, OUTPUT);
	push_button_leds[7].initialize(&expander_left, B, SEVEN, OUTPUT);

	// Right push-button LEDs
	push_button_leds[8].initialize(&expander_right, A, ONE, OUTPUT);
	push_button_leds[9].initialize(&expander_right, A, THREE, OUTPUT);
	push_button_leds[10].initialize(&expander_right, A, FIVE, OUTPUT);
	push_button_leds[11].initialize(&expander_right, A, SEVEN, OUTPUT);
	push_button_leds[12].initialize(&expander_right, B, ONE, OUTPUT);
	push_button_leds[13].initialize(&expander_right, B, THREE, OUTPUT);
	push_button_leds[14].initialize(&expander_right, B, FIVE, OUTPUT);
	push_button_leds[15].initialize(&expander_right, B, SEVEN, OUTPUT);
}

void OutputHandler::initialize_panel_leds ()
{
	panel_leds[0].initialize(C, ZERO, OUTPUT);
	panel_leds[1].initialize(C, ONE, OUTPUT);
	panel_leds[2].initialize(C, TWO, OUTPUT);
	panel_leds[3].initialize(C, THREE, OUTPUT);
	panel_leds[4].initialize(C, FOUR, OUTPUT);
	panel_leds[5].initialize(C, FIVE, OUTPUT);
	panel_leds[6].initialize(C, SIX, OUTPUT);
	panel_leds[7].initialize(C, SEVEN, OUTPUT);
	for (int i = 0; i < 8; ++i)
		panel_leds[i].write(0);
}
Clocks blinkTimer(500);
bool blinkState = false;
void OutputHandler::refresh_push_button_leds ()
{
	for (uint8_t i = 0; i < NUM_PUSH_BUTTONS; ++i)
	{
		if (memory->read(PUSH_BUTTON_0_FLAG+i) ) { //memory->read(MACRO_TYPE) - 1 == i) {
			if(memory->read(MACRO_TYPE))
			{
				push_button_leds[i].write(1);
			} else {
				if(blinkTimer.isDone())
				{
					push_button_leds[i].write(blinkState);
					blinkState^=1;
				}
			}
		}
		else {
			memory->write(PUSH_BUTTON_0_FLAG + i,0);
			push_button_leds[i].write(0);
		}
	}
}

void OutputHandler::refresh_panel_leds ()
{
	if (memory->read(CONNECTED))
	{
		panel_leds[0].write(1);
		panel_leds[4].write(0);
	}
	else
	{
		panel_leds[0].write(0);
		panel_leds[4].write(1);
	}

	if (memory->read(TIMEOUT_IN_PROGRESS))
	{
		panel_leds[3].write(0);
		panel_leds[7].write(1);
	}
	else
	{
		panel_leds[3].write(1);
		panel_leds[7].write(0);
	}

	if (memory->read(MACRO_TYPE))
	{
		panel_leds[2].write(0);
		panel_leds[6].write(1);
	}
	else
	{
		panel_leds[2].write(1);
		panel_leds[6].write(0);
	}
}

void OutputHandler::turn_off_push_button_leds ()
{
	for (int i = 0; i < NUM_PUSH_BUTTONS; ++i)
		push_button_leds[i].write(0);
}

void OutputHandler::spiral ()
{
	int indices [] = {0, 1, 8, 9, 11, 13, 15, 14, 7, 6, 4, 2, 3, 10, 12, 5};
	const int pause = 50;
	for (int j = 0; j < 16; ++j)
	{
		int index = indices[15 - j];
		push_button_leds[index].write(1);
		_delay_ms(pause);
	}
	for (int j = 0; j < 16; ++j)
	{
		int index = indices[15 - j];
		push_button_leds[index].write(0);
		_delay_ms(pause);
	}
}

void OutputHandler::drop ()
{
	int inner [] = {3, 5, 10, 12};
	int outer [] = {0, 1, 2, 4, 6, 7, 8, 9, 11, 13, 14, 15};
	int pause = 200;
	for (int j = 0; j < 4; ++j)
		push_button_leds[inner[j]].write(1);
	_delay_ms(pause);
	for (int j = 0; j < 12; ++j)
		push_button_leds[outer[j]].write(1);
	_delay_ms(pause);
	for (int j = 0; j < 4; ++j)
		push_button_leds[inner[j]].write(0);
	_delay_ms(pause);
	for (int j = 0; j < 12; ++j)
		push_button_leds[outer[j]].write(0);
	_delay_ms(pause);
}