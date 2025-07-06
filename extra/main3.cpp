#define F_CPU 1000000UL
#define BAUD 9600
#define UBRR_VALUE ((F_CPU / (16UL * BAUD)) - 1)  // UBRR = 6

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Buffer for received data
volatile unsigned char received_data;
volatile uint8_t data_ready = 0;

// Initialize UART
void uart_init(void) {
    UBRRH = (uint8_t)(UBRR_VALUE >> 8);
    UBRRL = (uint8_t)UBRR_VALUE;
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);  // 8N1
    UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);
    PORTD |= (1 << PD0);  // Enable internal pull-up on RX (PD0)
}

// Transmit a single character
void uart_transmit(unsigned char data) {
    while (!(UCSRA & (1 << UDRE)));
    UDR = data;
}

// Transmit a string
void uart_transmit_string(const unsigned char* str) {
    while (*str) {
        uart_transmit(*str++);
    }
}

// UART Receive Complete Interrupt
ISR(USART_RXC_vect) {
    if (!(UCSRA & (1 << RXC))) return;
    uint8_t status = UCSRA;
    if (status & (1 << FE) || status & (1 << DOR) || status & (1 << PE)) {
        uart_transmit_string("ERR\r\n");
        UDR;
        return;
    }
    received_data = UDR;
    if (received_data >= 32 && received_data <= 126) {  // Printable ASCII
        data_ready = 1;
    }
}

int main(void) {
    uart_init();
    _delay_ms(2000);  
    sei();
    uart_transmit_string("ATmega32 Bluetooth Echo Ready\r\n");

    while (1) {
        if (data_ready) {
            uart_transmit_string("RX: ");
            uart_transmit(received_data);
            uart_transmit('\r');
            uart_transmit('\n');
            data_ready = 0;
            _delay_ms(10);  // Throttle to prevent flooding
        }
        // Periodic test
        // uart_transmit_string("TEST\r\n");
        // _delay_ms(1000);
    }

    return 0;
}