#ifndef TOUCH_SCREEN_HPP
#define TOUCH_SCREEN_HPP
#include "../memory/memory.hpp"
#include "../uart/uart_module.hpp"

struct ScreenMessage
{
	uint8_t command;
	uint8_t object_type;
	uint8_t object_index;
	uint8_t value_msb;
	uint8_t value_lsb;
	uint8_t checksum;
	uint8_t calculate_correct_checksum ()
	{
		return command ^ object_type ^ object_index ^ value_msb ^ value_lsb;
	}
};
struct ScreenRequestMessage
{
	uint8_t command;
	uint8_t object_type;
	uint8_t object_index;

	uint8_t checksum;
	uint8_t calculate_correct_checksum ()
	{
		return command ^ object_type ^ object_index;
	}
};
#define FORM_0_C_count 1
#define FORM_1_C_count 20
#define FORM_2_C_count 6
#define FORM_3_C_count 10
const uint8_t FormControlCount[] = {FORM_0_C_count,FORM_1_C_count,FORM_2_C_count,FORM_3_C_count};
typedef struct {
	uint8_t Control;
	uint8_t ID;
	uint16_t Data;
} ScreenObject_t;

class TouchScreen
{
	public:
	void initialize (Memory *, UART_MODULE);
	void refresh () const;
	private:
	void assign_objects ();
	void UpdateObject_data(uint8_t) const;
	void assign_objects_for_test ();
	void update_object (ScreenObject_t) const;
	void update_object (uint16_t, uint16_t) const;
	int quary_object(uint8_t object) const;
	void log_string () const;
	void send_byte (uint8_t) const;
	bool send_message (const ScreenMessage &) const;
	bool send_message (const ScreenRequestMessage & msg) const;

	uint8_t read_byte () const;
	bool received_ack () const;
	private:
	uint16_t page;
	Memory * memory;
	UART_MODULE uart_module;
	uint16_t objects [Memory::SIZE];

};

#endif // TOUCH_SCREEN_HPP
