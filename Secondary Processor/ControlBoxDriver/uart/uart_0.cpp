#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "uart_0.hpp"

/*--------------------------- Constants and Macros ---------------------------*/

#define UART0_BUFFER_MASK (UART0_BUFFER_SIZE - 1)

#define UART0_RECEIVE_INTERRUPT   USART0_RX_vect
#define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
#define UART0_STATUS              UCSR0A
#define UART0_CONTROL_B           UCSR0B
#define UART0_CONTROL_C           UCSR0C  
#define UART0_DATA                UDR0

/*------------------------- Module Global Variables --------------------------*/

// Transmit Ring Buffer
static volatile uint8_t transmit_buffer [UART0_BUFFER_SIZE];
static volatile uint8_t transmit_head;
static volatile uint8_t transmit_tail;

// Receive Ring Buffer
static volatile uint8_t receive_buffer [UART0_BUFFER_SIZE];
static volatile uint8_t receive_head;
static volatile uint8_t receive_tail;

// Most Recent Receive Error
static volatile uint8_t last_receive_error;

/*------------------------ Interrupt Service Routines ------------------------*/

ISR (UART0_RECEIVE_INTERRUPT)
{
    uint8_t new_head, data, status, error;
    status = UART0_STATUS;
    data = UART0_DATA;
    
    // detect frame error, data overrun, or parity error
    error = status & ((1 << FE0) | (1 << DOR0)| (1 << UPE0));

    new_head = (receive_head + 1) & UART0_BUFFER_MASK;
    if (new_head == receive_tail)
        error = UART0_BUFFER_OVERFLOW >> 8;
    else
    {
        receive_head = new_head;
        receive_buffer[new_head] = data;
    }
    last_receive_error |= error;   
}

ISR (UART0_TRANSMIT_INTERRUPT)
{
    uint8_t new_tail;
    if (transmit_head != transmit_tail)
    {
        new_tail = (transmit_tail + 1) & UART0_BUFFER_MASK;
        transmit_tail = new_tail;
        UART0_DATA = transmit_buffer[new_tail]; // transmit
    }
    else
        UART0_CONTROL_B &= ~(1 << UDRIE0); // buffer empty so end transmission
}

/*------------------------------ Initialization ------------------------------*/

void uart0_initialize (uint16_t baudrate)
{
    transmit_head = 0;
    transmit_tail = 0;
    receive_head = 0;
    receive_tail = 0;

    UBRR0H = (uint8_t)((baudrate >> 8) & 0x80) ;
    UBRR0L = (uint8_t)(baudrate & 0x00FF);
      
    // set enable bits
    UART0_CONTROL_B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
    
    // set frame format (asynchronous, 8 bits of data, no parity bit, 1 stop bit)
    UART0_CONTROL_C = (1 << UCSZ01) | (1 << UCSZ00);
}

/*-------------------- Functions for Reading from Buffer ---------------------*/

uint16_t uart0_read_byte ()
{    
    uint8_t new_tail;
    uint8_t data;
    uint8_t error;
    if (receive_head == receive_tail)
        return UART0_NO_DATA;
    new_tail = (receive_tail + 1) & UART0_BUFFER_MASK;
    data = receive_buffer[new_tail];
    receive_tail = new_tail;
    error = last_receive_error;
    last_receive_error = 0;
    return (error << 8) + data;
}

/*--------------------- Functions for Writing to Buffer ----------------------*/

void uart0_send_byte (uint8_t data)
{
    uint8_t new_head;
    new_head  = (transmit_head + 1) & UART0_BUFFER_MASK;
    while (new_head == transmit_tail) {} // wait for free space in buffer
    transmit_buffer[new_head] = data;
    transmit_head = new_head;
    UART0_CONTROL_B |= (1 << UDRIE0); // enable data register interrupt
}

void uart0_send_string (const char * str)
{
	while (*str)
		uart0_send_byte(*str++);
}
