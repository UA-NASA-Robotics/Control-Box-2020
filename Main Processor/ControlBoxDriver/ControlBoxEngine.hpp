#ifndef CONTROL_BOX_ENGINE_HPP
#define CONTROL_BOX_ENGINE_HPP

#include "memory/memory.hpp"
#include "communications/communications.hpp"
#include "high_level_io/input_handler.hpp"
#include "i2c/SevenSegment.hpp"

class ControlBoxEngine
{
public:
	void initialize ();
	void loop ();
private:
	void listen_to_robot ();
	Memory memory;
	//Communications communications;
	Communications robot_FT;
	InputHandler inputs;
};

#endif // CONTROL_BOX_ENGINE_HPP