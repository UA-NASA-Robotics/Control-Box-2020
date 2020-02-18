#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "uart_1.hpp"

/*--------------------------- Constants and Macros ---------------------------*/

#define UART1_BUFFER_MASK (UART1_BUFFER_SIZE - 1)

#define UART1_RECEIVE_INTERRUPT   USART1_RX_vect
#define UART1_TRANSMIT_INTERRUPT  USART1_UDRE_vect
#define UART1_STATUS              UCSR1A
#define UART1_CONTROL_B           UCSR1B
#define UART1_CONTROL_C           UCSR1C
#define UART1_DATA                UDR1

/*------------------------- Module Global Variables --------------------------*/

// Transmit Ring Buffer
static volatile uint8_t transmit_buffer [UART1_BUFFER_SIZE];
static volatile uint8_t transmit_head;
static volatile uint8_t transmit_tail;

// Receive Ring Buffer
static volatile uint8_t receive_buffer [UART1_BUFFER_SIZE];
static volatile uint8_t receive_head;
static volatile uint8_t receive_tail;

// Most Recent Receive Error
static volatile uint8_t last_receive_error;

/*------------------------ Interrupt Service Routines ------------------------*/

ISR (UART1_RECEIVE_INTERRUPT)
{
	uint8_t new_head, data, status, error;
	status = UART1_STATUS;
	data = UART1_DATA;

	// detect frame error, data overrun, or parity error
	error = status & ((1 << FE1) | (1 << DOR1)| (1 << UPE1));

	new_head = (receive_head + 1) & UART1_BUFFER_MASK;
	if (new_head == receive_tail)
		error = UART1_BUFFER_OVERFLOW >> 8;
	else
	{
		receive_head = new_head;
		receive_buffer[new_head] = data;
	}
	last_receive_error |= error;
}

ISR (UART1_TRANSMIT_INTERRUPT)
{
	uint8_t new_tail;
	if (transmit_head != transmit_tail)
	{
		new_tail = (transmit_tail + 1) & UART1_BUFFER_MASK;
		transmit_tail = new_tail;
		UART1_DATA = transmit_buffer[new_tail]; // transmit
	}
	else
		UART1_CONTROL_B &= ~(1 << UDRIE1); // buffer empty so end transmission
}

/*------------------------------ Initialization ------------------------------*/

void uart1_initialize (uint16_t baudrate)
{
	transmit_head = 0;
	transmit_tail = 0;
	receive_head = 0;
	receive_tail = 0;

	UBRR1H = (uint8_t)((baudrate >> 8) & 0x80) ;
	UBRR1L = (uint8_t)(baudrate & 0x00FF);

	// set enable bits
	UART1_CONTROL_B = (1 << RXCIE1) | (1 << RXEN1) | (1 << TXEN1);

	// set frame format (asynchronous, 8 bits of data, no parity bit, 1 stop bit)
	UART1_CONTROL_C = (1 << UCSZ11) | (1 << UCSZ10);
}

/*-------------------- Functions for Reading from Buffer ---------------------*/

uint16_t uart1_read_byte ()
{
	uint8_t new_tail;
	uint8_t data;
	uint8_t error;
	if (receive_head == receive_tail)
		return UART1_NO_DATA;
	new_tail = (receive_tail + 1) & UART1_BUFFER_MASK;
	data = receive_buffer[new_tail];
	receive_tail = new_tail;
	error = last_receive_error;
	last_receive_error = 0;
	return (error << 8) + data;
}

/*--------------------- Functions for Writing to Buffer ----------------------*/

void uart1_send_byte (uint8_t data)
{
	uint8_t new_head;
	new_head  = (transmit_head + 1) & UART1_BUFFER_MASK;
	while (new_head == transmit_tail) {} // wait for free space in buffer
	transmit_buffer[new_head] = data;
	transmit_head = new_head;
	UART1_CONTROL_B |= (1 << UDRIE1); // enable data register interrupt
}

void uart1_send_string (const char * str)
{
	while (*str)
		uart1_send_byte(*str++);
}
