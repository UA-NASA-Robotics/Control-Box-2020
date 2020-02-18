#ifndef UART_3_H
#define UART_3_H

#include <stdint-gcc.h>

/*----------------------------- Set Buffer Sizes -----------------------------*/

// size of receive and transmit buffers (must be power of 2)
#define UART3_BUFFER_SIZE 32 

/*----------------------------- Baud Rate Setup ------------------------------*/

// See Table 22-1 (page 203) in datasheet for the following formula.

#define UART3_BAUD_SELECT(baud_rate,clock_rate) \
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
  // Evaluates to the value that should be passed to uart3_initialize in order
  // to initialize the USART Baud Rate Registers (UBBR3H and UBBR3L).
  //
  //////////////////////////////////////////////////////////////////////////////

/*------------------------------ Initialization ------------------------------*/

void uart3_initialize (uint16_t baudrate);
  //----------------------------------------------------------------------------
  // Pre-Conditions:
  //
  // Use the UART3_BAUD_SELECT macro to calculate 'baudrate' argument.
  //
  //----------------------------------------------------------------------------
  // Post-Conditions:
  //
  // UART3 module has been initialized with the given baudrate
  //
  //////////////////////////////////////////////////////////////////////////////

/*--------------------- Error Codes for uart2_read_byte ----------------------*/

#define UART3_FRAME_ERROR      0x1000 // framing error by uart
#define UART3_OVERRUN_ERROR    0x0800 // next byte arrived before UDR was read
#define UART3_PARITY_ERROR     0x0400 // next byte in buffer had parity error
#define UART3_BUFFER_OVERFLOW  0x0200 // receive buffer overflow error
#define UART3_NO_DATA          0x0100 // no receive data available

uint16_t uart3_read_byte ();
  //----------------------------------------------------------------------------
  // Pre-Conditions:
  //
  // UART module must already be initialized (by calling uart3_initialize)
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

void uart3_send_byte (uint8_t data);
  //----------------------------------------------------------------------------
  // Pre-Conditions:
  //
  // 1) UART module must already be initialized (by calling uart3_initialize)
  // 2) 'data' parameter is the next byte to be transmitted
  //
  //----------------------------------------------------------------------------
  // Post-Conditions:
  //
  // 'data' byte has been pushed to the back of the Transmit Ring Buffer
  //
  //////////////////////////////////////////////////////////////////////////////

void uart3_send_string (const char * str);
  //----------------------------------------------------------------------------
  // Pre-Conditions:
  //
  // 1) UART module must already be initialized (by calling uart3_initialize)
  // 2) 'str' must be a null-terminated c-string
  //
  //----------------------------------------------------------------------------
  // Post-Conditions:
  //
  // The string was transferred to the circular buffer and one character at
  // at time was transmitted to the UART3 module using interrupts.
  //
  //////////////////////////////////////////////////////////////////////////////

#endif // UART_3_H 
