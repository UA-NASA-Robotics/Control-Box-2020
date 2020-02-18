#ifndef UART_2_H
#define UART_2_H

#include <stdint-gcc.h>
#include <stdio.h>
#include <stdlib.h>
/*----------------------------- Set Buffer Sizes -----------------------------*/

// size of receive and transmit buffers (must be power of 2)
#define UART2_BUFFER_SIZE 32 

/*----------------------------- Baud Rate Setup ------------------------------*/

// See Table 22-1 (page 203) in datasheet for the following formula.

#define UART2_BAUD_SELECT(baud_rate,clock_rate) \
(((clock_rate) + 8UL * (baud_rate)) / (16UL * (baud_rate)) - 1UL)
  //----------------------------------------------------------------------------
  // Pre-Conditions:
  //
  // 1) 'baud_rate' is the target baud rate [bits/second]
  // 2) 'clock_rate' is the frequency of the system clock [MHz]
  //
  //----------------------------------------------------------------------------
  // Returns
  // 
  // Evaluates to the value that should be passed to uart2_initialize in order
  // to initialize the USART Baud Rate Registers (UBBR2H and UBBR2L).
  //
  //////////////////////////////////////////////////////////////////////////////

/*------------------------------ Initialization ------------------------------*/

void uart2_initialize (uint16_t baudrate);
  //----------------------------------------------------------------------------
  // Pre-Conditions:
  //
  // Use the UART2_BAUD_SELECT macro to calculate 'baudrate' argument.
  //
  //----------------------------------------------------------------------------
  // Post-Conditions:
  //
  // UART2 module has been initialized with the given baudrate
  //
  //////////////////////////////////////////////////////////////////////////////

/*--------------------- Error Codes for uart2_read_byte ----------------------*/

#define UART2_FRAME_ERROR      0x1000 // framing error by uart
#define UART2_OVERRUN_ERROR    0x0800 // next byte arrived before UDR was read
#define UART2_PARITY_ERROR     0x0400 // next byte in buffer had parity error
#define UART2_BUFFER_OVERFLOW  0x0200 // receive buffer overflow error
#define UART2_NO_DATA          0x0100 // no receive data available

uint16_t uart2_read_byte ();
  //----------------------------------------------------------------------------
  // Pre-Conditions:
  //
  // UART module must already be initialized (by calling uart2_initialize)
  //
  //----------------------------------------------------------------------------
  // Post-Conditions:
  //
  // One byte has been removed from the Receiving Ring Buffer
  //
  //----------------------------------------------------------------------------
  // Return Value:
  // 
  // Returns a 16 bit value.  The least significant byte contains the byte
  // that was read from the ring buffer, and the most significant byte
  // indicates either a successful or a failed read attempt. On failure,
  // the error codes above are used to indicate the cause of failure.
  //
  //////////////////////////////////////////////////////////////////////////////

void uart2_send_byte (uint8_t data);
  //----------------------------------------------------------------------------
  // Pre-Conditions:
  //
  // 1) UART module must already be initialized (by calling uart2_initialize)
  // 2) 'data' parameter is the next byte to be transmitted
  //
  //----------------------------------------------------------------------------
  // Post-Conditions:
  //
  // 'data' byte has been pushed to the back of the Transmit Ring Buffer
  //
  //////////////////////////////////////////////////////////////////////////////

void uart2_send_string (const char * str);
  //----------------------------------------------------------------------------
  // Pre-Conditions:
  //
  // 1) UART module must already be initialized (by calling uart2_initialize)
  // 2) 'str' must be a null-terminated c-string
  //
  //----------------------------------------------------------------------------
  // Post-Conditions:
  //
  // The string was transferred to the circular buffer and one character at
  // at time was transmitted to the UART2 module using interrupts.
  //
  //////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
extern "C"{
		// Needed for printf
		int uart_putchar(char c, FILE *stream);

		static FILE *mystdout;
}
#endif



#endif // UART_2_H 
