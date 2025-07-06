#include <avr/io.h>							/* Include AVR std. library file */
#include <util/delay.h>
#include <math.h>
#include <avr/interrupt.h>

#define F_CPU 16000000UL					/* Define CPU clock Frequency */
#define BAUDRATE 115200
#define DOUBLE_SPEED_MODE
#define STRING_BUFFER_SIZE 50
#define HEX_BYTE_BUFFER_SIZE 50

extern char string_buffer[STRING_BUFFER_SIZE]; /* Declare, don't define */
extern uint8_t hex_byte_buffer[HEX_BYTE_BUFFER_SIZE]; /* Declare, don't define */

extern volatile uint8_t string_ready; /* Declare as extern volatile */
extern volatile uint8_t string_buffer_index;
extern volatile uint8_t hex_byte_ready;
extern volatile uint8_t hex_byte_count;
extern volatile uint8_t hex_state; // 0: waiting for '0', 1: waiting for 'x', 2: first hex digit, 3: second hex digit
extern volatile uint8_t hex_nibble;

// Transmission mode
typedef enum {
    MODE_STRING, // ASCII string
    MODE_HEXOtt     // Hex byte array
} USART_TransmitMode;