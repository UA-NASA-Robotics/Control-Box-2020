#include "input_handler.hpp"
#include "../memory/memory_variables.hpp"
#include "../timer/Clock.hpp"
static const int NUM_ARCADE_BUTTONS = 9;
static const int NUM_PUSH_BUTTONS = 16;

Clocks RESET_TIMER(250);

Clocks PollInputs(100);
void InputHandler::initialize (Memory * memory)
{
	PollInputs.setInterval(100);
	this->memory = memory;
	initialize_expanders();
	initialize_push_buttons();
	initialize_arcade_buttons();
	initialize_joysticks();
	initialize_potentiometers();
}

void InputHandler::poll ()
{
	if(PollInputs.isDone()) {
		poll_arcade_buttons();
		//TODO: USE interrupt rather then polling
		poll_push_buttons();
		poll_analog_inputs();
	}
}

void InputHandler::clear_previous_inputs ()
{
	for (uint8_t i = PUSH_BUTTON_0_FLAG; i < PUSH_BUTTON_0_FLAG + NUM_PUSH_BUTTONS; ++i)
		memory->write(i, 0);
	for (uint8_t i = ARCADE_BUTTON_0_FLAG; i < ARCADE_BUTTON_0_FLAG + NUM_ARCADE_BUTTONS; ++i)
		memory->write(i, 0);
	memory->write(JOYSTICK_LEFT_X, 0);
	memory->write(JOYSTICK_LEFT_Y, 0);
	memory->write(JOYSTICK_RIGHT_X, 0);
	memory->write(JOYSTICK_RIGHT_Y, 0);
	memory->write(SLIDER_LEFT, 0);
	memory->write(SLIDER_RIGHT, 0);
}

void InputHandler::initialize_expanders ()
{
	expander_left.set_address(0x20);
	expander_right.set_address(0x27);
}

void InputHandler::initialize_push_buttons ()
{
	push_buttons[0].initialize(&expander_left, A, ZERO, INPUT);
	push_buttons[1].initialize(&expander_left, A, TWO,  INPUT);
	push_buttons[2].initialize(&expander_left, A, FOUR, INPUT);
	push_buttons[3].initialize(&expander_left, A, SIX,  INPUT);
	push_buttons[4].initialize(&expander_left, B, ZERO, INPUT);
	push_buttons[5].initialize(&expander_left, B, TWO,  INPUT);
	push_buttons[6].initialize(&expander_left, B, FOUR, INPUT);
	push_buttons[7].initialize(&expander_left, B, SIX,  INPUT);
	push_buttons[8].initialize(&expander_right, A, ZERO, INPUT);
	push_buttons[9].initialize(&expander_right, A, TWO,  INPUT);
	push_buttons[10].initialize(&expander_right, A, FOUR, INPUT);
	push_buttons[11].initialize(&expander_right, A, SIX,  INPUT);
	push_buttons[12].initialize(&expander_right, B, ZERO, INPUT);
	push_buttons[13].initialize(&expander_right, B, TWO,  INPUT);
	push_buttons[14].initialize(&expander_right, B, FOUR, INPUT);
	push_buttons[15].initialize(&expander_right, B, SIX,  INPUT);
}

void InputHandler::initialize_arcade_buttons ()
{
	arcade_buttons[0].initialize(K, TWO, INPUT_PULLUP);   // emergency stop
	arcade_buttons[1].initialize(B, SIX, INPUT_PULLUP);   // n/a
	arcade_buttons[2].initialize(B, FOUR, INPUT_PULLUP);  // n/a
	arcade_buttons[3].initialize(B, ZERO, INPUT_PULLUP);  // arm up
	arcade_buttons[4].initialize(E, FOUR, INPUT_PULLUP);  // lower bucket
	arcade_buttons[5].initialize(B, FIVE, INPUT_PULLUP);  // raise bucket
	arcade_buttons[6].initialize(E, SIX, INPUT_PULLUP);   // arm down
	arcade_buttons[7].initialize(E, SEVEN, INPUT_PULLUP); // plow up
	arcade_buttons[8].initialize(E, FIVE, INPUT_PULLUP);  // plow down
}

void InputHandler::initialize_joysticks ()
{
	joystick_left_y.initialize(CHANNEL_ZERO);
	joystick_left_x.initialize(CHANNEL_ONE);
	joystick_right_x.initialize(CHANNEL_SIX);
	joystick_right_y.initialize(CHANNEL_SEVEN);
}

void InputHandler::initialize_potentiometers ()
{
	potentiometer_left.initialize(CHANNEL_TWO);
	potentiometer_right.initialize(CHANNEL_FIVE);
}

void InputHandler::poll_arcade_buttons ()
{
	for (uint8_t i = 0; i < NUM_ARCADE_BUTTONS; ++i)
		memory->write(ARCADE_BUTTON_0_FLAG + i, arcade_buttons[i].is_low() ? 1 : 0);
}

void InputHandler::poll_push_buttons ()
{
	for (uint8_t i = 0; i < NUM_PUSH_BUTTONS; ++i)
	{

		if (!push_buttons[i].read()) {
			memory->write(PUSH_BUTTON_0_FLAG + i, 1);
			//RESET_TIMER.reset();
		}
//    else {
//      if(RESET_TIMER.isDone_NoReset())
//        memory->write(PUSH_BUTTON_0_FLAG + i, 0);
//    }
	}
}

void InputHandler::poll_analog_inputs ()
{
	memory->write(JOYSTICK_LEFT_X,  joystick_left_x.read());
	memory->write(JOYSTICK_LEFT_Y,  joystick_left_y.read());
	memory->write(JOYSTICK_RIGHT_X, joystick_right_x.read());
	memory->write(JOYSTICK_RIGHT_Y, joystick_right_y.read());
	memory->write(SLIDER_LEFT,      potentiometer_left.read());
	memory->write(SLIDER_RIGHT,     potentiometer_right.read());
}
