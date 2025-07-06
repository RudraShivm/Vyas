#include <avr/io.h>
#include <param.h>
#include <string.h>
#include "USART_RS232_H_file.h"	/* include USART library */

#define LED PORTB		/* connected LED on PORT pin */
char string_buffer[STRING_BUFFER_SIZE];
uint8_t hex_byte_buffer[HEX_BYTE_BUFFER_SIZE];
int main(void)
{
    DDRB = 0xff;		/* make PORT as output port */
    USART_Init();		/* initialize USART with 115200 baud rate */
    LED = 0;			/* initialize LED off */
    sei();				/* enable global interrupts for USART RX */
    
    while (1)
    {
        /* Check if a string is ready */
        if (USART_IsStringReady()) {
            char* received_string = USART_GetStringBuffer();
            if (strcmp(received_string, "ON") == 0) {
                LED |= (1 << PB0); /* Turn ON LED */
                USART_SendString("LED turned ON\r\n");
            } else if (strcmp(received_string, "OFF") == 0) {
                LED &= ~(1 << PB0); /* Turn OFF LED */
                USART_SendString("LED turned OFF\r\n");
            } else {
                USART_SendString("Received string: ");
                USART_SendString(received_string);
                USART_SendString("\r\n");
            }
            USART_ClearStringBuffer();
            USART_SetStringReady(0);
        }
        
        /* Check if hex data is ready */
        if (USART_IsHexReady()) {
            uint8_t* hex_buffer = USART_GetHexBuffer();
            uint8_t hex_size = USART_GetHexBufferSize();
            
            if (hex_size > 0) {
                if (hex_buffer[0] == 0x01) {
                    LED |= (1 << PB0); /* Turn ON LED for 0x01 */
                    USART_SendString("LED turned ON (hex: 0x01)\r\n");
                } else if (hex_buffer[0] == 0x02) {
                    LED &= ~(1 << PB0); /* Turn OFF LED for 0x02 */
                    USART_SendString("LED turned OFF (hex: 0x02)\r\n");
                } else {
                    USART_SendString("Received hex: ");
                    USART_SendHexByte(hex_buffer, hex_size);
                    USART_SendString("\r\n");
                }
            }
            USART_ClearHexBuffer();
            USART_SetHexReady(0);
        }
    }
}

uint8_t hex_to_nibble(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0'; // Convert digits 0-9 to 0x0-0x9
    } else if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10; // Convert A-F to 0xA-0xF
    } else if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10; // Convert a-f to 0xA-0xF
    }
    return 0; // Return 0 for invalid input
}


ISR(USART_RXC_vect) {
    char received = UDR;
    USART_TxChar(received);
    // String reception (for debugging or other purposes)
    if (string_buffer_index < STRING_BUFFER_SIZE - 1 && !string_ready) {
        if (received == '\n' || received == '\r') {
            string_buffer[string_buffer_index] = '\0';
            string_ready = 1;
            string_buffer_index = 0;
        } else {
            string_buffer[string_buffer_index++] = received;
        }
    }
    
    // Hex parsing state machine
    if (hex_byte_count < HEX_BYTE_BUFFER_SIZE) {
        switch (hex_state) {
            case 0: // Waiting for '0'
                if (received == '0') hex_state = 1;
                break;
            case 1: // Waiting for 'x'
                if (received == 'x') hex_state = 2;
                else hex_state = 0;
                break;
            case 2: // First hex digit
                if ((received >= '0' && received <= '9') || (received >= 'A' && received <= 'F') || (received >= 'a' && received <= 'f')) {
                    hex_nibble = hex_to_nibble(received) << 4;
                    hex_state = 3;
                } else {
                    hex_state = 0;
                }
                break;
            case 3: // Second hex digit
                if ((received >= '0' && received <= '9') || (received >= 'A' && received <= 'F') || (received >= 'a' && received <= 'f')) {
                    hex_byte_buffer[hex_byte_count++] = hex_nibble | hex_to_nibble(received);
                    hex_byte_ready = 1;
                    string_ready = 0;
                    hex_state = 0; // Reset for next hex value
                } else {
                    hex_state = 0;
                }
                break;
        }
    }
}