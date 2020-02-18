#ifndef MEMORY_HPP
#define MEMORY_HPP

#include <stdint-gcc.h>

/*--------------------------------- Variable ---------------------------------*/

struct Variable
{
	void initialize ();
	uint16_t value;
	Variable * next;
	Variable * prev;
};

/*----------------------------- Memory Interface -----------------------------*/

class Memory
{
public:
	static const uint16_t SIZE = 128;
	static void initialize ();
	static void write (uint8_t, uint16_t);
	static uint16_t read (uint8_t);
	static void link (uint8_t, uint8_t);
	static bool valid_address (uint8_t);
};

#endif // MEMORY_HPP
