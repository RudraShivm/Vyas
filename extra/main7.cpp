#include <avr/io.h>
#include <param.h>
#include <string.h>
#include <stdio.h>
#include <PololuQTRSensors.h>
#include "USART_RS232_H_file.h"
#include <util/delay.h>

#define LEFT_MOTOR_PWM 19   // PD5 (OC1A) for left motor PWM (PWMA)
#define RIGHT_MOTOR_PWM 18  // PD4 (OC1B) for right motor PWM (PWMB)
#define LEFT_MOTOR_DIR2 22  // PC0 for left motor direction (BIN2)
#define LEFT_MOTOR_DIR1 23  // PC1 for left motor direction (BIN1)
#define MOTOR_STBY 24       // PC2 for TB6612FNG standby control
#define RIGHT_MOTOR_DIR1 25 // PC3 for right motor direction (AIN1)
#define RIGHT_MOTOR_DIR2 26 // PC4 for right motor direction (AIN2)
#define LEFT_MOTOR_MAX 245  // calibrated value
#define RIGHT_MOTOR_MAX 255
#define LED PORTB /* connected LED on PORT pin */
#define EMITTER_PIN_PORT PORTC
#define EMITTER_PIN PC7
#define EMITTER_PIN_DDR DDRC
#define CALIBRATION_TIME_MS 5000 // 5 seconds for calibration
#define NUM_SENSORS 8
// PID constants
#define KP 0.5                    // Proportional gain for PID control
#define KD 2.0                    // Derivative gain for PID control
#define BASE_SPEED 150            // Base PWM value for motor speed (0-255)
#define LINE_POSITION_CENTER 3500 // Center position for 8 sensors (0–7000)
#define BLACK_THRESHOLD 600       // Threshold for detecting black line
#define NO_SURFACE_THRESHOLD 950  // Threshold for detecting no surface
#define HALF_TURN_DELAY 200       // Turning 90 Degree delay

// Calibration data structure
typedef struct {
    uint16_t minValues[NUM_SENSORS];
    uint16_t maxValues[NUM_SENSORS];
} CalibrationData;

// Operating modes
enum Mode
{
    CMD,
    AUTONOMOUS
};
volatile Mode robotMode = CMD; // Start in CMD mode

char string_buffer[STRING_BUFFER_SIZE];
uint8_t hex_byte_buffer[HEX_BYTE_BUFFER_SIZE];
volatile uint8_t string_ready = 0;
volatile uint8_t string_buffer_index = 0;
volatile uint8_t hex_byte_ready = 0;
volatile uint8_t hex_byte_count = 0;
volatile uint8_t hex_state = 0;
volatile uint8_t hex_nibble = 0;

volatile uint16_t g_leftSpeed = 0;
volatile uint16_t g_rightSpeed = 0;
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
    if (leftSpeed >= 0)
    {
        PORTC |= (1 << (LEFT_MOTOR_DIR2 - 22));  // AIN1 high (forward)
        PORTC &= ~(1 << (LEFT_MOTOR_DIR1 - 22)); // AIN2 low
    }
    else
    {
        PORTC &= ~(1 << (LEFT_MOTOR_DIR2 - 22)); // AIN1 low (reverse)
        PORTC |= (1 << (LEFT_MOTOR_DIR1 - 22));  // AIN2 high
        leftSpeed = -leftSpeed;                  // Convert to positive speed
    }
    // Right Motor (Motor B)
    if (rightSpeed >= 0)
    {
        PORTC |= (1 << (RIGHT_MOTOR_DIR1 - 22));  // BIN1 high (forward)
        PORTC &= ~(1 << (RIGHT_MOTOR_DIR2 - 22)); // BIN2 low
    }
    else
    {
        PORTC &= ~(1 << (RIGHT_MOTOR_DIR1 - 22)); // BIN1 low (reverse)
        PORTC |= (1 << (RIGHT_MOTOR_DIR2 - 22));  // BIN2 high
        rightSpeed = -rightSpeed;                 // Convert to positive speed
    }
    if (leftSpeed > LEFT_MOTOR_MAX)
        leftSpeed = LEFT_MOTOR_MAX;
    if (rightSpeed > RIGHT_MOTOR_MAX)
        rightSpeed = RIGHT_MOTOR_MAX;
    if (LEFT_MOTOR_PWM == 19)
    {
        OCR1A = leftSpeed;
        g_leftSpeed = leftSpeed;
    }
    if (RIGHT_MOTOR_PWM == 18)
    {
        OCR1B = rightSpeed;
        g_rightSpeed = rightSpeed;
    }
}

void ADC_Init(void)
{
    // Set ADC reference to AVCC, left-adjust disabled
    ADMUX = (1 << REFS0);
    // Enable ADC, prescaler 128 (156.25 kHz @ 20 MHz)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t ADC_Read(uint8_t channel)
{
    // Select ADC channel (0–7 for PA0–PA7)
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    // Start conversion
    ADCSRA |= (1 << ADSC);
    // Wait for conversion to complete
    while (ADCSRA & (1 << ADSC));
    // Return 10-bit ADC value
    return ADC;
}

void QTR_EmitterOn(void)
{
    EMITTER_PIN_PORT |= (1 << EMITTER_PIN); // Set PD0 high
    _delay_us(200); // Wait for sensors to stabilize
}

void QTR_EmitterOff(void)
{
    EMITTER_PIN_PORT &= ~(1 << EMITTER_PIN); // Set PD0 low
    _delay_us(200); // Wait for sensors to stabilize
}
// Perform automated calibration by moving robot in zigzag pattern
void calibrateSensors(CalibrationData* calData)
{
    // Initialize min and max values
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        calData->minValues[i] = 1023; // Max ADC value
        calData->maxValues[i] = 0;    // Min ADC value
    }

    // Send calibration start message
    USART_SendString("Calibrating sensors... Move over line and surface\r\n");

    // Calibration loop for 5 seconds
    uint32_t startTime = 0; // Simplified timer (assumes 16 MHz clock)
    while (startTime < CALIBRATION_TIME_MS) {
        QTR_EmitterOn();
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
            uint16_t value = ADC_Read(i);
            if (value < calData->minValues[i]) calData->minValues[i] = value;
            if (value > calData->maxValues[i]) calData->maxValues[i] = value;
        }
        QTR_EmitterOff();
        _delay_ms(20); // Sample every 20ms
        startTime += 20;
    }

    // Send calibration complete message with min/max values
    char buf[32];
    USART_SendString("Calibration complete. Min/Max values:\r\n");
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        snprintf(buf, sizeof(buf), "S%d: %d/%d ", i, calData->minValues[i], calData->maxValues[i]);
        USART_SendString(buf);
    }
    USART_SendString("\r\n");
}

void readCalibrated(uint16_t* sensorValues, uint16_t* calibratedValues, CalibrationData* calData)
{
    QTR_EmitterOn();
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        sensorValues[i] = ADC_Read(i);
        // Normalize to 0–1000
        if (calData->maxValues[i] > calData->minValues[i]) {
            char buf[32];
            snprintf(buf, sizeof(buf), "sensorVal: %d\r\n",sensorValues[i]);
            USART_SendString(buf);
            snprintf(buf, sizeof(buf), "calMax: %d, calMin: %d, sen: %d\r\n", calData->maxValues[i], calData->minValues[i], sensorValues[i]);
            USART_SendString(buf);
            calibratedValues[i] = ((uint32_t)(sensorValues[i] - calData->minValues[i]) * 1000) /
                                 (calData->maxValues[i] - calData->minValues[i]);
            snprintf(buf, sizeof(buf), "caliVAl: %d\r\n", calibratedValues[i]);
            USART_SendString(buf);
        } else {
            calibratedValues[i] = 0; // Avoid division by zero
        }
    }
    QTR_EmitterOff();
}

uint16_t readLinePosition(uint16_t* calibratedValues, uint8_t numSensors)
{
    uint32_t avg = 0;
    uint16_t sum = 0;
    uint8_t onLine = 0;

    for (uint8_t i = 0; i < numSensors; i++) {
        uint16_t value = calibratedValues[i];
        // Consider sensor "on line" if reading is above threshold (normalized scale)
        if (value > 500) // Threshold for normalized values (tune if needed)
        {
            avg += (uint32_t)value * (i * 1000);
            sum += value;
            onLine = 1;
        }
    }

    // If no line detected, return center position (3500)
    if (!onLine) {
        return 3500;
    }

    // Calculate weighted average for line position (0 to 7000)
    return (uint16_t)(avg / sum);
}

void followLine(PololuQTRSensorsAnalog &qtr)
{
    static int16_t lastError = 0;
    unsigned int sensorValues[8];
    qtr.readCalibrated(sensorValues, QTR_EMITTERS_ON_AND_OFF);

    // Count sensors detecting black
    uint8_t blackSensors = 0;
    uint8_t noSurfaceSensors = 0;
    bool sensorState[8];
    uint8_t minBlackSensorIdx = 8;
    uint8_t maxBlackSensorIdx = -1;
    for (uint8_t i = 0; i < 8; i++)
    {
        if (sensorValues[i] > BLACK_THRESHOLD)
        {
            blackSensors++;
            sensorState[i] = true;
            if (i < minBlackSensorIdx)
                minBlackSensorIdx = i;
            if (i < maxBlackSensorIdx)
                maxBlackSensorIdx = i;
        }
        if (sensorValues[i] > NO_SURFACE_THRESHOLD)
        {
            noSurfaceSensors++;
        }
    }
    // Logic based on number of sensors detecting black
    if (noSurfaceSensors > 0) // Priority: Detect no surface first
    {
        setMotorSpeeds(0, 0); // Stop robot
        USART_SendString("No surface detected, stopped\r\n");
    }
    else if (blackSensors > 5)
    {
        setMotorSpeeds(0, 0); // Stop robot
        USART_SendString("Victory\r\n");
    }
    else if (blackSensors >= 4)
    {
        if (maxBlackSensorIdx + minBlackSensorIdx <= 10 && maxBlackSensorIdx + minBlackSensorIdx >= 4)
        {
            // if the line is junction given that avg of max and min black sensor idx is towards mid
            setMotorSpeeds(0, 0); // Stop robot
        }
        else
        {
            // else we probably encountered a turn in normal line and we have to keep following
            unsigned int position = qtr.readLine(sensorValues, QTR_EMITTERS_ON_AND_OFF, 0);
            int16_t error = position - LINE_POSITION_CENTER;
            int16_t correction = KP * error + KD * (error - lastError);
            lastError = error;
            int16_t calibration_factor = LEFT_MOTOR_MAX - RIGHT_MOTOR_MAX;
            int16_t leftSpeed = BASE_SPEED + calibration_factor + correction;
            int16_t rightSpeed = BASE_SPEED - correction;
            setMotorSpeeds(leftSpeed, rightSpeed);
        }
    }
    else if (blackSensors >= 2 || blackSensors < 4)
    {
        // when at least 2 sensor senses black, keeping the detection at reasonable width for avoiding noise
        unsigned int position = qtr.readLine(sensorValues, QTR_EMITTERS_ON_AND_OFF, 0);
        int16_t error = position - LINE_POSITION_CENTER;
        int16_t correction = KP * error + KD * (error - lastError);
        lastError = error;
        int16_t calibration_factor = LEFT_MOTOR_MAX - RIGHT_MOTOR_MAX;
        int16_t leftSpeed = BASE_SPEED + calibration_factor + correction;
        int16_t rightSpeed = BASE_SPEED - correction;
        setMotorSpeeds(leftSpeed, rightSpeed);
    }
    else
    {
        // only 1 sensor is sensing black, in this case, stop the robot
        setMotorSpeeds(0, 0);
    }
}

const char* getDirection(uint16_t* calibratedValues, uint8_t numSensors)
{
    uint16_t leftSum = 0, centerSum = 0, rightSum = 0;
    
    // Sum calibrated values for each region
    for (uint8_t i = 0; i < numSensors; i++) {
        if (i <= 2) // Sensors 0–2 (Left)
            leftSum += calibratedValues[i];
        else if (i <= 4) // Sensors 3–4 (Center)
            centerSum += calibratedValues[i];
        else // Sensors 5–7 (Right)
            rightSum += calibratedValues[i];
    }

    // Find the region with the highest sum (indicating line presence)
    if (leftSum > centerSum && leftSum > rightSum)
        return "Left";
    else if (rightSum > leftSum && rightSum > centerSum)
        return "Right";
    else
        return "Center";
}

int main(void)
{
    DDRB = 0xff;  /* make PORT as output port */
    LED = 0;      /* initialize LED off */
    USART_Init(); /* initialize USART with 115200 baud rate */
    sei();        /* enable global interrupts for USART RX */
    initPWM();
    ADC_Init();
    MCUCSR |= (1 << JTD); // Disable JTAG
    MCUCSR |= (1 << JTD); // Write twice in the same cycle
    EMITTER_PIN_DDR |= (1 << EMITTER_PIN); // Set emitter pin output
    CalibrationData calData;
    uint16_t sensorValues[NUM_SENSORS];
    uint16_t calibratedValues[NUM_SENSORS];
    char buf[32];
    setMotorSpeeds(0, 0);

    while (1)
    {
        /* Check if a string is ready */
        if (USART_IsStringReady())
        {
            char *received = USART_GetStringBuffer();
            if (strlen(received) == 1)
            {
                char cmd = received[0];
                if (robotMode == CMD)
                {
                    switch (cmd)
                    {
                    case 'R':
                        setMotorSpeeds(LEFT_MOTOR_MAX / 2, 0);
                        break;
                    case 'L':
                        setMotorSpeeds(0, RIGHT_MOTOR_MAX / 2);
                        break;
                    case 'F':
                        setMotorSpeeds(LEFT_MOTOR_MAX / M_2_PI, RIGHT_MOTOR_MAX / 2);
                        break;
                    case 'S':
                        // slow
                        setMotorSpeeds(LEFT_MOTOR_MAX / 4, RIGHT_MOTOR_MAX / 4);
                        break;
                    case 'B':
                        setMotorSpeeds(0, 0);
                        break;
                    default:
                        USART_SendString("Please switch to CMD mode first"); //?
                        USART_SendString("\r\n");
                        break;
                    }
                }
                switch (cmd)
                {
                case 'C':
                    calibrateSensors(&calData);
                    break;
                case 'K':
                    readCalibrated(sensorValues, calibratedValues, &calData);
                    const char* direction = getDirection(calibratedValues, NUM_SENSORS);
                    snprintf(buf, sizeof(buf), "Dir: %s\r\n",  direction);
                    USART_SendString(buf);
                    break;
                case 'T':
                    if (robotMode == AUTONOMOUS)
                    {
                        USART_SendString("Switching to CMD mode");
                        USART_SendString("\r\n");
                        robotMode = CMD;
                    }
                    else
                    {
                        USART_SendString("Switching to AUTONOMOUS mode");
                        USART_SendString("\r\n");
                        robotMode = AUTONOMOUS;
                    }
                    break;
                default:
                    USART_SendString("Received string: ");
                    USART_SendString(received);
                    USART_SendString("\r\n");
                    break;
                }
                USART_ClearStringBuffer();
                USART_SetStringReady(0);

                /* Check if hex data is ready */
                if (USART_IsHexReady())
                {
                    uint8_t *hex_buffer = USART_GetHexBuffer();
                    uint8_t hex_size = USART_GetHexBufferSize();

                    if (hex_size > 0)
                    {
                        if (g_leftSpeed == 0 && g_rightSpeed == 0)
                        {
                            if (hex_buffer[0] >> 6 == 0x00)
                            {
                                // turn right
                                setMotorSpeeds(BASE_SPEED + LEFT_MOTOR_MAX - RIGHT_MOTOR_MAX, BASE_SPEED);
                                _delay_ms(100);
                                setMotorSpeeds(BASE_SPEED + LEFT_MOTOR_MAX - RIGHT_MOTOR_MAX, 0);
                                _delay_ms(HALF_TURN_DELAY);
                                setMotorSpeeds(BASE_SPEED + LEFT_MOTOR_MAX - RIGHT_MOTOR_MAX, BASE_SPEED);
                                _delay_ms(100);
                            }
                            else if (hex_buffer[0] >> 6 == 0x01)
                            {
                                // go forward
                                setMotorSpeeds(BASE_SPEED + LEFT_MOTOR_MAX - RIGHT_MOTOR_MAX, BASE_SPEED);
                                _delay_ms(100);
                            }
                            else if (hex_buffer[0] >> 6 == 0x02)
                            {
                                // turn left
                                setMotorSpeeds(BASE_SPEED + LEFT_MOTOR_MAX - RIGHT_MOTOR_MAX, BASE_SPEED);
                                _delay_ms(100);
                                setMotorSpeeds(0, BASE_SPEED);
                                _delay_ms(HALF_TURN_DELAY);
                                setMotorSpeeds(BASE_SPEED + LEFT_MOTOR_MAX - RIGHT_MOTOR_MAX, BASE_SPEED);
                                _delay_ms(100);
                            }
                            else if (hex_buffer[0] >> 6 == 0x03)
                            {
                                // go back
                                setMotorSpeeds(BASE_SPEED + LEFT_MOTOR_MAX - RIGHT_MOTOR_MAX, BASE_SPEED);
                                _delay_ms(100);
                                setMotorSpeeds(0, BASE_SPEED);
                                _delay_ms(HALF_TURN_DELAY * 2);
                                setMotorSpeeds(BASE_SPEED + LEFT_MOTOR_MAX - RIGHT_MOTOR_MAX, BASE_SPEED);
                                _delay_ms(100);
                            }
                            else
                            {
                                USART_SendString("Received hex: ");
                                USART_SendHexByte(hex_buffer, hex_size);
                                USART_SendString("\r\n");
                            }
                        }
                    }
                    USART_ClearHexBuffer();
                    USART_SetHexReady(0);
                }
                if (robotMode == AUTONOMOUS)
                {
                    // followLine(qt);
                }
                // else
                // {
                //     setMotorSpeeds(0, 0);
                // }
            }
        }
    }
}

uint8_t hex_to_nibble(char c)
{
    if (c >= '0' && c <= '9')
    {
        return c - '0'; // Convert digits 0-9 to 0x0-0x9
    }
    else if (c >= 'A' && c <= 'F')
    {
        return c - 'A' + 10; // Convert A-F to 0xA-0xF
    }
    else if (c >= 'a' && c <= 'f')
    {
        return c - 'a' + 10; // Convert a-f to 0xA-0xF
    }
    return 0; // Return 0 for invalid input
}

ISR(USART_RXC_vect)
{
    char received = UDR;
    USART_TxChar(received);
    // String reception (for debugging or other purposes)
    if (string_buffer_index < STRING_BUFFER_SIZE - 1 && !string_ready)
    {
        if (received == '\n' || received == '\r')
        {
            string_buffer[string_buffer_index] = '\0';
            string_ready = 1;
            string_buffer_index = 0;
        }
        else
        {
            string_buffer[string_buffer_index++] = received;
        }
    }

    // Hex parsing state machine
    if (hex_byte_count < HEX_BYTE_BUFFER_SIZE)
    {
        switch (hex_state)
        {
        case 0: // Waiting for '0'
            if (received == '0')
                hex_state = 1;
            break;
        case 1: // Waiting for 'x'
            if (received == 'x')
                hex_state = 2;
            else
                hex_state = 0;
            break;
        case 2: // First hex digit
            if ((received >= '0' && received <= '9') || (received >= 'A' && received <= 'F') || (received >= 'a' && received <= 'f'))
            {
                hex_nibble = hex_to_nibble(received) << 4;
                hex_state = 3;
            }
            else
            {
                hex_state = 0;
            }
            break;
        case 3: // Second hex digit
            if ((received >= '0' && received <= '9') || (received >= 'A' && received <= 'F') || (received >= 'a' && received <= 'f'))
            {
                hex_byte_buffer[hex_byte_count++] = hex_nibble | hex_to_nibble(received);
                hex_byte_ready = 1;
                hex_state = 0; // Reset for next hex value
            }
            else
            {
                hex_state = 0;
            }
            break;
        }
    }
}