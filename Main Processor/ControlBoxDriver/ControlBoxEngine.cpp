#include "ControlBoxEngine.hpp"
#include <avr/interrupt.h>
#include "timer/Clock.hpp"
#include "digital_io/digital_pin.hpp"

void ControlBoxEngine::initialize ()
{
	sei();
	memory.initialize();
	inputs.initialize(&memory);
	communications.initialize(UART_1, &memory);
}
void ControlBoxEngine::loop ()
{
	while (1)
	{
		communications.check_connection();
		inputs.poll();
		communications.transmit();
		communications.receive();
		
	}
}
