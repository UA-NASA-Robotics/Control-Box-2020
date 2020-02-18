#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "uart_3.hpp"
#include "uart_2.hpp"

/*--------------------------- Constants and Macros ---------------------------*/

#define UART3_BUFFER_MASK (UART3_BUFFER_SIZE - 1)

#define UART3_RECEIVE_INTERRUPT   USART3_RX_vect
#define UART3_TRANSMIT_INTERRUPT  USART3_UDRE_vect
#define UART3_STATUS              UCSR3A
#define UART3_CONTROL_B           UCSR3B
#define UART3_CONTROL_C           UCSR3C
#define UART3_DATA                UDR3

/*------------------------- Module Global Variables --------------------------*/

// Transmit Ring Buffer
static volatile uint8_t transmit_buffer [UART3_BUFFER_SIZE];
static volatile uint8_t transmit_head;
static volatile uint8_t transmit_tail;

// Receive Ring Buffer
static volatile uint8_t receive_buffer [UART3_BUFFER_SIZE];
static volatile uint8_t receive_head;
static volatile uint8_t receive_tail;

// Most Recent Receive Error
static volatile uint8_t last_receive_error;

/*------------------------ Interrupt Service Routines ------------------------*/

ISR (UART3_RECEIVE_INTERRUPT)
{
	uint8_t new_head, data, status, error;
	status = UART3_STATUS;
	data = UART3_DATA;

	// detect frame error, data overrun, or parity error
	error = status & ((1 << FE3) | (1 << DOR3)| (1 << UPE3));

	new_head = (receive_head + 1) & UART3_BUFFER_MASK;
	if (new_head == receive_tail)
		error = UART3_BUFFER_OVERFLOW >> 8;
	else
	{
		receive_head = new_head;
		receive_buffer[new_head] = data;
	}
	last_receive_error |= error;
}

ISR (UART3_TRANSMIT_INTERRUPT)
{
	uint8_t new_tail;
	if (transmit_head != transmit_tail)
	{
		new_tail = (transmit_tail + 1) & UART3_BUFFER_MASK;
		transmit_tail = new_tail;
		UART3_DATA = transmit_buffer[new_tail]; // transmit
	}
	else
		UART3_CONTROL_B &= ~(1 << UDRIE3); // buffer empty so end transmission
}

/*------------------------------ Initialization ------------------------------*/

void uart3_initialize (uint16_t baudrate)
{
	transmit_head = 0;
	transmit_tail = 0;
	receive_head = 0;
	receive_tail = 0;

	UBRR3H = (uint8_t)((baudrate >> 8) & 0x80) ;
	UBRR3L = (uint8_t)(baudrate & 0x00FF);

	// set enable bits
	UART3_CONTROL_B = (1 << RXCIE3) | (1 << RXEN3) | (1 << TXEN3);

	// set frame format (asynchronous, 8 bits of data, no parity bit, 1 stop bit)
	UART3_CONTROL_C = (1 << UCSZ31) | (1 << UCSZ30);
}

/*-------------------- Functions for Reading from Buffer ---------------------*/

uint16_t uart3_read_byte ()
{
	uint8_t new_tail;
	uint8_t data;
	uint8_t error;
	if (receive_head == receive_tail)
		return UART3_NO_DATA;
	new_tail = (receive_tail + 1) & UART3_BUFFER_MASK;
	data = receive_buffer[new_tail];
	receive_tail = new_tail;
	error = last_receive_error;
	last_receive_error = 0;
	return (error << 8) + data;
}

/*--------------------- Functions for Writing to Buffer ----------------------*/

void uart3_send_byte (uint8_t data)
{
	uint8_t new_head;
	new_head  = (transmit_head + 1) & UART3_BUFFER_MASK;
	while (new_head == transmit_tail) {} // wait for free space in buffer
	transmit_buffer[new_head] = data;
	transmit_head = new_head;
	UART3_CONTROL_B |= (1 << UDRIE3); // enable data register interrupt
	//if(data == 0x85)
	//printf("data:%d\r\n",data);
}

void uart3_send_string (const char * str)
{
	while (*str)
		uart3_send_byte(*str++);
}
