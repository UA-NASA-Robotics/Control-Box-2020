#include "i2c.hpp"
#include <util/twi.h>
#include "../uart/uart_3.hpp"
#include <stdio.h>

// Usage of delay is in "fail" function (allow time to transmit stop condition)
#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include <util/delay.h>

// Used to make switches cleaner:
#define when break;case
#define otherwise break;default

/*--------------------------- Baud Rate Parameters ---------------------------*/

#define CLOCK_SPEED 16000000UL
#define F_SCL 100000UL // set baud rate here
#define PRESCALE 1
#define TWBR_VALUE ((((CLOCK_SPEED / F_SCL) / PRESCALE) - 16) / 2)

/*----------- Status Codes (returned by "write" & "read" methods) ------------*/

#define I2C_SUCCESS 1 // status code returned upon successful read/write
#define I2C_FAILURE 0 // status code returned upon failed read/write

/*----------------- Maximum Number of Failures before Abort ------------------*/

#define MAX_FAIL_COUNT 5

/*------------------------ External Helper Functions -------------------------*/

void copy_range (uint8_t * src, uint8_t * dest, uint8_t size)
{
	for (int i = 0; i < size; ++i)
		*dest++ = *src++;
}

bool interrupt_flag_set ()
{
	return TWCR & (1 << TWINT);
}

bool check_status (uint8_t status)
{
	return (TWSR & 0xF8) == status;
}

/*----------------------------- Public Interface -----------------------------*/

void I2C::set_address (uint8_t _address)
{
	this->address = _address << 1;
}

int I2C::write (uint8_t * data, uint8_t size)
{

	copy_range(data, buffer, size);
	reset(size);
	while (state != DONE)
	{
		if (fail_count >= MAX_FAIL_COUNT)
			return I2C_FAILURE;
		switch (state)
		{
		case START:
			start();
when VERIFY_START:
			verify_start();
when SEND_ADDRESS:
			send_address(WRITE);
when VERIFY_ADDRESS:
			verify_write_address_sent();
when SEND_BYTE:
			send_data_byte(data[index]);
when VERIFY_DATA:
			verify_data_sent();
when STOP:
			stop();
otherwise:
			state = DONE;
		}
	}
	return I2C_SUCCESS;
}

int I2C::read (uint8_t * data, uint8_t size)
{
	reset(size);
	while (state != DONE)
	{
		if (fail_count >= MAX_FAIL_COUNT)
			return I2C_FAILURE;
		switch (state)
		{
		case START:
			start();
when VERIFY_START:
			verify_start();
when SEND_ADDRESS:
			send_address(READ);
when VERIFY_ADDRESS:
			verify_read_address_sent();
when READ_BYTE:
			read_data_byte();
when VERIFY_DATA:
			verify_data_read();
when STOP:
			stop();
otherwise:
			state = DONE;
		}
	}
	copy_range(buffer, data, size);
	return I2C_SUCCESS;
}

/*------------------------ Internal Helper Functions -------------------------*/

void I2C::reset (uint8_t size)
{
	TWBR = (uint8_t)TWBR_VALUE;
	this->size = size;
	index = 0;
	fail_count = 0;
	state = START;
}

void I2C::start ()
{
	TWCR = (1 << TWSTA) | (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
	state = VERIFY_START;
}

void I2C::verify_start ()
{
	if (!interrupt_flag_set())
		return;
	if (check_status(TW_START))
		state = SEND_ADDRESS;
	else
		fail();
}

void I2C::send_address (I2C::Direction direction)
{
	//printf("I2Cwriteaddress: %d\n",address);
	if (direction == READ)
		address |= 0x01;
	else
		address &= 0xFE;
	TWDR = address;
	TWCR = (1 << TWINT) | (1 << TWEN);
	state = VERIFY_ADDRESS;
}

void I2C::verify_write_address_sent ()
{
	if (interrupt_flag_set())
	{
		if (check_status(TW_MT_SLA_ACK))
			state = SEND_BYTE;
		else
			fail();
	}
}

void I2C::verify_read_address_sent ()
{
	if (interrupt_flag_set())
	{
		if (check_status(TW_MR_SLA_ACK))
			state = READ_BYTE;
		else
			fail();
	}
}

void I2C::send_data_byte (uint8_t data)
{
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	state = VERIFY_DATA;
}

void I2C::read_data_byte ()
{
	if (index == size - 1)
		TWCR ^= (1 << TWEA); // send NACK after reading last byte
	TWCR = (1 << TWINT) | (1 << TWEN);
	state = VERIFY_DATA;
}

void I2C::verify_data_sent ()
{
	if (!interrupt_flag_set())
		return;
	if (check_status(TW_MT_DATA_ACK))
	{
		if (++index < size)
			state = SEND_BYTE;
		else
			state = STOP;
	}
	else
		fail();
}

void I2C::verify_data_read ()
{
	if (!interrupt_flag_set())
		return;
	bool last_byte = !(TWCR & (1 << TWEA));
	uint8_t status = last_byte ? TW_MR_DATA_NACK : TW_MR_DATA_ACK;
	State next_state = last_byte ? STOP : READ_BYTE;
	if (check_status(status))
	{
		buffer[index++] = TWDR;
		state = next_state;
	}
	else
		fail();
}

void I2C::stop ()
{
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
	state = DONE;
}

void I2C::fail ()
{
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
	index = 0;
	state = START;
	++fail_count;
	_delay_us(10);
}

