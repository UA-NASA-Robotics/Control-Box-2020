

#ifndef SIMPLE_TIMER_1
#define SIMPLE_TIMER_1
#include <stdint-gcc.h>
class SimpleTimer1
{
public:
	SimpleTimer1 ();
	SimpleTimer1 (int l);
	void start ();
	void restart ();
	void set_duration (uint32_t);
	bool is_finished ();
	void reset ();
	uint32_t millis();
};

#endif 