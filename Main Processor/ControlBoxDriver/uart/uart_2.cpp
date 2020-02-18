#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>
#include "uart_2.hpp"

/*--------------------------- Constants and Macros ---------------------------*/

#define UART2_BUFFER_MASK (UART2_BUFFER_SIZE - 1)

#define UART2_RECEIVE_INTERRUPT   USART2_RX_vect
#define UART2_TRANSMIT_INTERRUPT  USART2_UDRE_vect
#define UART2_STATUS              UCSR2A
#define UART2_CONTROL_B           UCSR2B
#define UART2_CONTROL_C           UCSR2C
#define UART2_DATA                UDR2

/*------------------------- Module Global Variables --------------------------*/

// Transmit Ring Buffer
static volatile uint8_t transmit_buffer [UART2_BUFFER_SIZE];
static volatile uint8_t transmit_head;
static volatile uint8_t transmit_tail;

// Receive Ring Buffer
static volatile uint8_t receive_buffer [UART2_BUFFER_SIZE];
static volatile uint8_t receive_head;
static volatile uint8_t receive_tail;

// Most Recent Receive Error
static volatile uint8_t last_receive_error;

/*------------------------ Interrupt Service Routines ------------------------*/

ISR (UART2_RECEIVE_INTERRUPT)
{
	uint8_t new_head, data, status, error;
	status = UART2_STATUS;
	data = UART2_DATA;

	// detect frame error, data overrun, or parity error
	error = status & ((1 << FE2) | (1 << DOR2)| (1 << UPE2));

	new_head = (receive_head + 1) & UART2_BUFFER_MASK;
	if (new_head == receive_tail)
		error = UART2_BUFFER_OVERFLOW >> 8;
	else
	{
		receive_head = new_head;
		receive_buffer[new_head] = data;
	}
	last_receive_error |= error;
}

ISR (UART2_TRANSMIT_INTERRUPT)
{
	uint8_t new_tail;
	if (transmit_head != transmit_tail)
	{
		new_tail = (transmit_tail + 1) & UART2_BUFFER_MASK;
		transmit_tail = new_tail;
		UART2_DATA = transmit_buffer[new_tail]; // transmit
	}
	else
		UART2_CONTROL_B &= ~(1 << UDRIE2); // buffer empty so end transmission
}

/*------------------------------ Initialization ------------------------------*/

void uart2_initialize (uint16_t baudrate)
{
	transmit_head = 0;
	transmit_tail = 0;
	receive_head = 0;
	receive_tail = 0;

	UBRR2H = (uint8_t)((baudrate >> 8) & 0x80) ;
	UBRR2L = (uint8_t)(baudrate & 0x00FF);

	// set enable bits
	UART2_CONTROL_B = (1 << RXCIE2) | (1 << RXEN2) | (1 << TXEN2);

	// set frame format (asynchronous, 8 bits of data, no parity bit, 1 stop bit)
	UART2_CONTROL_C = (1 << UCSZ21) | (1 << UCSZ20);
	// For printf Functionality
	mystdout = fdevopen(uart_putchar, NULL);
	stdout = mystdout;
}

/*-------------------- Functions for Reading from Buffer ---------------------*/

uint16_t uart2_read_byte ()
{
	uint8_t new_tail;
	uint8_t data;
	uint8_t error;
	if (receive_head == receive_tail)
		return UART2_NO_DATA;
	new_tail = (receive_tail + 1) & UART2_BUFFER_MASK;
	data = receive_buffer[new_tail];
	receive_tail = new_tail;
	error = last_receive_error;
	last_receive_error = 0;
	return (error << 8) + data;
}

/*--------------------- Functions for Writing to Buffer ----------------------*/

void uart2_send_byte (uint8_t data)
{
	uint8_t new_head;
	new_head  = (transmit_head + 1) & UART2_BUFFER_MASK;
	while (new_head == transmit_tail) {} // wait for free space in buffer
	transmit_buffer[new_head] = data;
	transmit_head = new_head;
	UART2_CONTROL_B |= (1 << UDRIE2); // enable data register interrupt
}

void uart2_send_string (const char * str)
{
	while (*str)
		uart2_send_byte(*str++);
}


int uart_putchar(char c, FILE *stream)
{

	if (c != '\n') {
		uart2_send_byte(c);
	} else {
		uart2_send_byte('\r');
	}

	return 0;
}