#include "ControlBoxEngine.hpp"
#include <avr/interrupt.h>
#include "timer/Clock.hpp"
#include "digital_io/digital_pin.hpp"
#include "uart/uart_2.hpp"

void ControlBoxEngine::initialize ()
{
	sei();
	uart2_initialize(UART2_BAUD_SELECT(115200UL, 16000000UL));  // For printf
	memory.initialize();
	
	outputs.initialize(&memory);
	communications.initialize(UART_1, &memory);
}
void ControlBoxEngine::loop ()
{
	outputs.startup();

	while (1)
	{
		outputs.refresh();
		communications.receive();
	}
}
