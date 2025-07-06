#include <avr/io.h>
#include <param.h>
#include <string.h>
#include "USART_RS232_H_file.h"	/* include USART library */

#define LEFT_MOTOR_PWM  19  // PD5 (OC1A) for left motor PWM (PWMA)
#define RIGHT_MOTOR_PWM 18  // PD4 (OC1B) for right motor PWM (PWMB)
#define LEFT_MOTOR_DIR2 22  // PC0 for left motor direction (BIN2)
#define LEFT_MOTOR_DIR1 23  // PC1 for left motor direction (BIN1)
#define MOTOR_STBY      24  // PC2 for TB6612FNG standby control
#define RIGHT_MOTOR_DIR1 25 // PC3 for right motor direction (AIN1)
#define RIGHT_MOTOR_DIR2 26 // PC4 for right motor direction (AIN2)
#define LEFT_MOTOR_MAX 245
#define RIGHT_MOTOR_MAX 255
#define LED PORTB		/* connected LED on PORT pin */

char string_buffer[STRING_BUFFER_SIZE];
uint8_t hex_byte_buffer[HEX_BYTE_BUFFER_SIZE];
volatile uint8_t string_ready = 0;
volatile uint8_t string_buffer_index = 0;
volatile uint8_t hex_byte_ready = 0;
volatile uint8_t hex_byte_count = 0;
volatile uint8_t hex_state = 0;
volatile uint8_t hex_nibble = 0;

// Initialize Timer1 for PWM and set pin directions
void initPWM()
{
    // Set PWM pins (PD5, PD4) as outputs
    DDRD |= (1 << (LEFT_MOTOR_PWM - 14)) | (1 << (RIGHT_MOTOR_PWM - 14));
    // Set direction pins (PD1, PD6, PD2, PD7) as outputs
    DDRC |= (1 << (LEFT_MOTOR_DIR1 - 22)) | (1 << (LEFT_MOTOR_DIR2 - 22)) | 
            (1 << (RIGHT_MOTOR_DIR1 - 22)) | (1 << (RIGHT_MOTOR_DIR2 - 22));
    // Set standby pin (PD3) as output
    DDRC |= (1 << (MOTOR_STBY - 22));
    // Enable driver by setting STBY high
    PORTC |= (1 << (MOTOR_STBY - 22));
    // Initialize Timer1 for both micros() and PWM
    TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1); // 8-bit Fast PWM, non-inverting mode
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);     // Prescaler 64
}

// Set motor speeds with direction control for TB6612FNG
void setMotorSpeeds(int16_t leftSpeed, int16_t rightSpeed)
{
    // Left Motor (Motor A)
    if (leftSpeed >= 0) {
        PORTC |= (1 << (LEFT_MOTOR_DIR2 - 22));  // AIN1 high (forward)
        PORTC &= ~(1 << (LEFT_MOTOR_DIR1 - 22)); // AIN2 low
    } else {
        PORTC &= ~(1 << (LEFT_MOTOR_DIR2 - 22)); // AIN1 low (reverse)
        PORTC |= (1 << (LEFT_MOTOR_DIR1 - 22));  // AIN2 high
        leftSpeed = -leftSpeed;                  // Convert to positive speed
    }
    // Right Motor (Motor B)
    if (rightSpeed >= 0) {
        PORTC |= (1 << (RIGHT_MOTOR_DIR1 - 22));  // BIN1 high (forward)
        PORTC &= ~(1 << (RIGHT_MOTOR_DIR2 - 22)); // BIN2 low
    } else {
        PORTC &= ~(1 << (RIGHT_MOTOR_DIR1 - 22)); // BIN1 low (reverse)
        PORTC |= (1 << (RIGHT_MOTOR_DIR2 - 22));  // BIN2 high
        rightSpeed = -rightSpeed;                // Convert to positive speed
    }
    if (leftSpeed > LEFT_MOTOR_MAX) leftSpeed = LEFT_MOTOR_MAX;    // Limit left speed to 8-bit PWM max
    if (rightSpeed > RIGHT_MOTOR_MAX) rightSpeed = RIGHT_MOTOR_MAX;  // Limit right speed to 8-bit PWM max
    if (LEFT_MOTOR_PWM == 19) OCR1A = leftSpeed;  // Set left motor PWM (OC1A, PWMA)
    if (RIGHT_MOTOR_PWM == 18) OCR1B = rightSpeed; // Set right motor PWM (OC1B, PWMB)
}

int main(void)
{
    DDRB = 0xff;		/* make PORT as output port */
    USART_Init();		/* initialize USART with 115200 baud rate */
    LED = 0;			/* initialize LED off */
    sei();				/* enable global interrupts for USART RX */
    initPWM(); 
    setMotorSpeeds(30, 30);
    while (1)
    {
        /* Check if a string is ready */
        if (USART_IsStringReady()) {
            USART_TxChar('i');
            char* received_string = USART_GetStringBuffer();
            if (strcmp(received_string, "right") == 0) {
                // LED |= (1 << PB0); /* Turn ON LED */
                // USART_SendString("LED turned ON\r\n");
                // setMotorSpeeds(LEFT_MOTOR_MAX-20, RIGHT_MOTOR_MAX);
                setMotorSpeeds(LEFT_MOTOR_MAX/2, 0);
            } else if (strcmp(received_string, "left") == 0) {
                // LED &= ~(1 << PB0); /* Turn OFF LED */
                // USART_SendString("LED turned OFF\r\n");
                // setMotorSpeeds(LEFT_MOTOR_MAX, RIGHT_MOTOR_MAX-20);
                setMotorSpeeds(0, RIGHT_MOTOR_MAX/2);
            } else if (strcmp(received_string, "forward") == 0) { 
                setMotorSpeeds(LEFT_MOTOR_MAX/2, RIGHT_MOTOR_MAX/2);
            } else if (strcmp(received_string, "slow") == 0) { 
                setMotorSpeeds(LEFT_MOTOR_MAX/4, RIGHT_MOTOR_MAX/4);
            } else if (strcmp(received_string, "break") == 0) { 
                setMotorSpeeds(0, 0);
            }else {
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
                    hex_state = 0; // Reset for next hex value
                } else {
                    hex_state = 0;
                }
                break;
        }
    }
}