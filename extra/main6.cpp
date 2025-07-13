#include <avr/io.h>
#include <param.h>
#include <string.h>
#include <stdio.h>
#include <PololuQTRSensors.h>
#include "USART_RS232_H_file.h"

#define LEFT_MOTOR_PWM  19  // PD5 (OC1A) for left motor PWM (PWMA)
#define RIGHT_MOTOR_PWM 18  // PD4 (OC1B) for right motor PWM (PWMB)
#define LEFT_MOTOR_DIR2 22  // PC0 for left motor direction (BIN2)
#define LEFT_MOTOR_DIR1 23  // PC1 for left motor direction (BIN1)
#define MOTOR_STBY      24  // PC2 for TB6612FNG standby control
#define RIGHT_MOTOR_DIR1 25 // PC3 for right motor direction (AIN1)
#define RIGHT_MOTOR_DIR2 26 // PC4 for right motor direction (AIN2)
#define LEFT_MOTOR_MAX  245 // Calibrated max PWM for left motor
#define RIGHT_MOTOR_MAX 255 // Calibrated max PWM for right motor
#define LED             PORTB // Connected LED on PORTB
#define EMITTER_PIN     29    // Emitter pin for QTR-8A
// PID constants
#define KP              0.5   // Proportional gain
#define KD              2.0   // Derivative gain
#define BASE_SPEED      150   // Base PWM speed (0–255)
#define LINE_POSITION_CENTER 3500 // Center position for 8 sensors (0–7000)
#define BLACK_THRESHOLD 600   // Threshold for detecting black line

// Operating modes
enum Mode { MANUAL, AUTONOMOUS };
volatile Mode robotMode = MANUAL; // Start in manual mode

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
    DDRD |= (1 << (LEFT_MOTOR_PWM - 14)) | (1 << (RIGHT_MOTOR_PWM - 14));
    DDRC |= (1 << (LEFT_MOTOR_DIR1 - 22)) | (1 << (LEFT_MOTOR_DIR2 - 22)) | 
            (1 << (RIGHT_MOTOR_DIR1 - 22)) | (1 << (RIGHT_MOTOR_DIR2 - 22));
    DDRC |= (1 << (MOTOR_STBY - 22));
    PORTC |= (1 << (MOTOR_STBY - 22)); // Enable driver
    TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1); // 8-bit Fast PWM
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);     // Prescaler 64
}

// Set motor speeds with direction control for TB6612FNG
void setMotorSpeeds(int16_t leftSpeed, int16_t rightSpeed)
{
    if (leftSpeed >= 0) {
        PORTC |= (1 << (LEFT_MOTOR_DIR2 - 22));
        PORTC &= ~(1 << (LEFT_MOTOR_DIR1 - 22));
    } else {
        PORTC &= ~(1 << (LEFT_MOTOR_DIR2 - 22));
        PORTC |= (1 << (LEFT_MOTOR_DIR1 - 22));
        leftSpeed = -leftSpeed;
    }
    if (rightSpeed >= 0) {
        PORTC |= (1 << (RIGHT_MOTOR_DIR1 - 22));
        PORTC &= ~(1 << (RIGHT_MOTOR_DIR2 - 22));
    } else {
        PORTC &= ~(1 << (RIGHT_MOTOR_DIR1 - 22));
        PORTC |= (1 << (RIGHT_MOTOR_DIR2 - 22));
        rightSpeed = -rightSpeed;
    }
    if (leftSpeed > LEFT_MOTOR_MAX) leftSpeed = LEFT_MOTOR_MAX;
    if (rightSpeed > RIGHT_MOTOR_MAX) rightSpeed = RIGHT_MOTOR_MAX;
    if (LEFT_MOTOR_PWM == 19) OCR1A = leftSpeed;
    if (RIGHT_MOTOR_PWM == 18) OCR1B = rightSpeed;
}

// Perform automated calibration by moving robot in zigzag pattern
void calibrateRobot(PololuQTRSensorsAnalog &qtr)
{
    qtr.resetCalibration(); // Clear previous calibration
    for (uint8_t i = 0; i < 50; i++) {
        setMotorSpeeds(100, -100); // Turn left
        qtr.calibrate(QTR_EMITTERS_ON_AND_OFF);
        _delay_ms(20);
    }
    for (uint8_t i = 0; i < 50; i++) {
        setMotorSpeeds(-100, 100); // Turn right
        qtr.calibrate(QTR_EMITTERS_ON_AND_OFF);
        _delay_ms(20);
    }
    for (uint8_t i = 0; i < 50; i++) {
        setMotorSpeeds(100, 100); // Move straight
        qtr.calibrate(QTR_EMITTERS_ON_AND_OFF);
        _delay_ms(20);
    }
    setMotorSpeeds(0, 0); // Stop robot
    USART_SendString("Calibration Complete\r\n");
}

// Follow line using PID control or stop/send victory based on sensor count
void followLine(PololuQTRSensorsAnalog &qtr)
{
    static int16_t lastError = 0;
    unsigned int sensorValues[8];
    // Read calibrated sensor values
    qtr.readCalibrated(sensorValues, QTR_EMITTERS_ON_AND_OFF);
    
    // Count sensors detecting black
    uint8_t blackSensors = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (sensorValues[i] > BLACK_THRESHOLD) {
            blackSensors++;
        }
    }

    // Logic based on number of sensors detecting black
    if (blackSensors > 4) {
        setMotorSpeeds(0, 0); // Stop robot
        USART_SendString("Victory\r\n");
        robotMode = MANUAL; // Switch to manual mode
    } else if (blackSensors == 4) {
        setMotorSpeeds(0, 0); // Stop robot
    } else if (blackSensors == 2) {
        // Perform PID line-following
        unsigned int position = qtr.readLine(sensorValues, QTR_EMITTERS_ON_AND_OFF, 0);
        int16_t error = position - LINE_POSITION_CENTER;
        int16_t correction = KP * error + KD * (error - lastError);
        lastError = error;
        int16_t leftSpeed = BASE_SPEED + correction;
        int16_t rightSpeed = BASE_SPEED - correction;
        setMotorSpeeds(leftSpeed, rightSpeed);
    } else {
        // If not exactly 2 sensors, maintain last motor speeds (or stop if desired)
        // setMotorSpeeds(0, 0); // Optional: Stop if not 2 sensors
    } 
}

int main(void)
{
    DDRB = 0xFF; // PORTB as output for LED
    LED = 0;     // Initialize LED off
    USART_Init(); // Initialize USART at 115200 baud
    sei();       // Enable global interrupts for USART RX
    initPWM();   // Initialize PWM for motors

    // Initialize QTR-8A sensors
    uint8_t analogPins[] = {40, 39, 38, 37, 36, 35, 34, 33};
    PololuQTRSensorsAnalog qtr(analogPins, 8, 4, EMITTER_PIN);

    while (1)
    {
        // Check for Bluetooth commands
        if (USART_IsStringReady()) {
            char* received = USART_GetStringBuffer();
            if (strlen(received) == 1) { // Expect single-character commands
                char cmd = received[0];
                switch (cmd) {
                    case 'F': // Forward
                        robotMode = MANUAL;
                        setMotorSpeeds(LEFT_MOTOR_MAX / 2, RIGHT_MOTOR_MAX / 2);
                        USART_SendString("Moving Forward\r\n");
                        break;
                    case 'L': // Left
                        robotMode = MANUAL;
                        setMotorSpeeds(0, RIGHT_MOTOR_MAX / 2);
                        USART_SendString("Turning Left\r\n");
                        break;
                    case 'R': // Right
                        robotMode = MANUAL;
                        setMotorSpeeds(LEFT_MOTOR_MAX / 2, 0);
                        USART_SendString("Turning Right\r\n");
                        break;
                    case 'S': // Slow
                        robotMode = MANUAL;
                        setMotorSpeeds(LEFT_MOTOR_MAX / 4, RIGHT_MOTOR_MAX / 4);
                        USART_SendString("Moving Slow\r\n");
                        break;
                    case 'B': // Brake
                        robotMode = MANUAL;
                        setMotorSpeeds(0, 0);
                        USART_SendString("Stopped\r\n");
                        break;
                    case 'C': // Calibrate
                        robotMode = MANUAL;
                        calibrateRobot(qtr);
                        break;
                    case 'A': // Autonomous line-following
                        robotMode = AUTONOMOUS;
                        USART_SendString("Autonomous Mode\r\n");
                        break;
                    default:
                        USART_SendString("Unknown command: ");
                        USART_SendString(received);
                        USART_SendString("\r\n");
                }
            } else {
                USART_SendString("Invalid command length\r\n");
            }
            USART_ClearStringBuffer();
            USART_SetStringReady(0);
        }

        // Check for hex data (e.g., for LED control)
        if (USART_IsHexReady()) {
            uint8_t* hex_buffer = USART_GetHexBuffer();
            uint8_t hex_size = USART_GetHexBufferSize();
            if (hex_size > 0) {
                if (hex_buffer[0] == 0x01) {
                    LED |= (1 << PB0);
                    USART_SendString("LED ON (hex: 0x01)\r\n");
                } else if (hex_buffer[0] == 0x02) {
                    LED &= ~(1 << PB0);
                    USART_SendString("LED OFF (hex: 0x02)\r\n");
                } else {
                    USART_SendString("Received hex: ");
                    USART_SendHexByte(hex_buffer, hex_size);
                    USART_SendString("\r\n");
                }
            }
            USART_ClearHexBuffer();
            USART_SetHexReady(0);
        }

        // Autonomous line-following mode
        if (robotMode == AUTONOMOUS) {
            followLine(qtr);
        }
    }
}

// Hex to nibble conversion for hex parsing
uint8_t hex_to_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

// USART RX interrupt handler
ISR(USART_RXC_vect) {
    char received = UDR;
    USART_TxChar(received); // Echo received character
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
                if ((received >= '0' && received <= '9') || 
                    (received >= 'A' && received <= 'F') || 
                    (received >= 'a' && received <= 'f')) {
                    hex_nibble = hex_to_nibble(received) << 4;
                    hex_state = 3;
                } else {
                    hex_state = 0;
                }
                break;
            case 3: // Second hex digit
                if ((received >= '0' && received <= '9') || 
                    (received >= 'A' && received <= 'F') || 
                    (received >= 'a' && received <= 'f')) {
                    hex_byte_buffer[hex_byte_count++] = hex_nibble | hex_to_nibble(received);
                    hex_byte_ready = 1;
                    hex_state = 0;
                } else {
                    hex_state = 0;
                }
                break;
        }
    }
}