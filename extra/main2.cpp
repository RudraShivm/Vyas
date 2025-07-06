#include <QTRSensors.h>
#include <USART_RS232_H_file.h>
#include <stdio.h>

// Motor control pins
#define LEFT_MOTOR_PWM  19  // PD5 (OC1A) for left motor PWM (PWMA)
#define RIGHT_MOTOR_PWM 18  // PD4 (OC1B) for right motor PWM (PWMB)
#define LEFT_MOTOR_DIR1 25  // PD1 for left motor direction (AIN1)
#define LEFT_MOTOR_DIR2 26  // PD6 for left motor direction (AIN2)
#define RIGHT_MOTOR_DIR1 23 // PD2 for right motor direction (BIN1)
#define RIGHT_MOTOR_DIR2 22 // PD7 for right motor direction (BIN2)
#define MOTOR_STBY      24  // PD3 for TB6612FNG standby control
#define EMITTER_PIN     29  // PD0 for QTR sensor emitter control

// PID constants
#define KP 0.5           // Proportional gain for PID control
#define KD 2.0           // Derivative gain for PID control
#define BASE_SPEED 150   // Base PWM value for motor speed (0-255)

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
        PORTC |= (1 << (LEFT_MOTOR_DIR1 - 22));  // AIN1 high (forward)
        PORTC &= ~(1 << (LEFT_MOTOR_DIR2 - 22)); // AIN2 low
    } else {
        PORTC &= ~(1 << (LEFT_MOTOR_DIR1 - 22)); // AIN1 low (reverse)
        PORTC |= (1 << (LEFT_MOTOR_DIR2 - 22));  // AIN2 high
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
    if (leftSpeed > 255) leftSpeed = 255;    // Limit left speed to 8-bit PWM max
    if (rightSpeed > 255) rightSpeed = 255;  // Limit right speed to 8-bit PWM max
    if (LEFT_MOTOR_PWM == 19) OCR1A = leftSpeed;  // Set left motor PWM (OC1A, PWMA)
    if (RIGHT_MOTOR_PWM == 18) OCR1B = rightSpeed; // Set right motor PWM (OC1B, PWMB)
}

// Perform automated calibration by moving robot in zigzag pattern
void calibrateRobot(QTRSensors &qtr)
{
    for (uint8_t i = 0; i < 50; i++) {
        setMotorSpeeds(100, -100);             // Turn left for calibration
        qtr.calibrate(QTRReadMode::OnAndOff);  // Calibrate with emitters on and off
        _delay_ms(20);                         // Delay for movement
    }
    for (uint8_t i = 0; i < 50; i++) {
        setMotorSpeeds(-100, 100);             // Turn right for calibration
        qtr.calibrate(QTRReadMode::OnAndOff);  // Calibrate with emitters on and off
        _delay_ms(20);                         // Delay for movement
    }
    for (uint8_t i = 0; i < 50; i++) {
        setMotorSpeeds(100, 100);              // Move straight for calibration
        qtr.calibrate(QTRReadMode::OnAndOff);  // Calibrate with emitters on and off
        _delay_ms(20);                         // Delay for movement
    }
    setMotorSpeeds(0, 0);                      // Stop robot after calibration
    USART_SendString("Calibration Complete\r\n"); // Notify remote device
}

// Main program loop for LFR operation with USART feedback
int main(void)
{
    USART_Init();                             // Initialize HC-05 Bluetooth module
    sei();                                    // Enable global interrupts
    USART_SendString("ATmega32A HC-05 Ready\r\n"); // Initial status message

    initPWM();                                // Set up PWM for motor control

    QTRSensors qtr;                           // Create QTR sensor object
    qtr.setTypeAnalog();                      // Configure for analog sensors
    uint8_t sensorPins[] = {40, 39, 38, 37, 36, 35, 34, 33}; // PA0-PA7 (pins 40-33)
    qtr.setSensorPins(sensorPins, 8);         // Set sensor pins
    qtr.setEmitterPin(EMITTER_PIN);           // Set emitter control pin

    calibrateRobot(qtr);                      // Perform initial calibration

    uint16_t sensorValues[8];                 // Array to store sensor readings
    int16_t lastError = 0;                    // Previous error for PID derivative

    while (1) {
        // Use OnAndOff mode for line following to match calibration
        uint16_t position = qtr.readLineBlack(sensorValues, QTRReadMode::OnAndOff);
        int16_t error = position - 3500;      // Calculate error from center (3500 for 8 sensors)
        int16_t derivative = error - lastError; // Calculate derivative for PID
        int16_t motorAdjustment = (KP * error) + (KD * derivative); // PID adjustment
        int16_t leftSpeed = BASE_SPEED - motorAdjustment;  // Adjust left motor speed
        int16_t rightSpeed = BASE_SPEED + motorAdjustment; // Adjust right motor speed
        setMotorSpeeds(leftSpeed, rightSpeed); // Apply motor speeds
        lastError = error;                    // Update last error

        // Send sensor data and position to remote device
        USART_SendString("Sensor Values: ");
        for (uint8_t i = 0; i < 8; i++) {
            char buf[5];
            snprintf(buf, sizeof(buf), "%d ", sensorValues[i]);
            USART_SendString(buf);
        }
        USART_SendString("\r\nPosition: ");
        char posBuf[6];
        snprintf(posBuf, sizeof(posBuf), "%d\r\n", position);
        USART_SendString(posBuf);

        // Check and handle received data from remote device
        if (USART_IsHexReady()) {
            USART_SendString("Received Hex: ");
            USART_SendHexByte(USART_GetHexBuffer(), USART_GetHexBufferSize());
            USART_SendString("\r\n");
            USART_ClearHexBuffer();
            USART_SetHexReady(0); // Clear flag
        } else if (USART_IsStringReady()) {
            USART_SendString("Received String: ");
            USART_SendString(USART_GetStringBuffer());
            USART_SendString("\r\n");
            USART_ClearStringBuffer();
            USART_SetStringReady(0); // Clear flag
        }

        _delay_ms(1000);                       // Delay for stability and periodic status
        USART_SendString("Status: OK\r\n");    // Periodic status update
    }

    return 0;                                 // Unreachable, included for completeness
}