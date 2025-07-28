#include <avr/io.h>
#include <param.h>
#include <string.h>
#include <stdio.h>
#include <PololuQTRSensors.h>
#include "USART_RS232_H_file.h"
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <inttypes.h>
#define EEPROM_MAGIC_NUMBER 0xA5A5 // Unique identifier to check if EEPROM is initialized

#define LEFT_MOTOR_PWM 19   // PD5 (OC1A) for left motor PWM (PWMA)
#define RIGHT_MOTOR_PWM 18  // PD4 (OC1B) for right motor PWM (PWMB)
#define LEFT_MOTOR_DIR2 22  // PC0 for left motor direction (BIN2)
#define LEFT_MOTOR_DIR1 23  // PC1 for left motor direction (BIN1)
#define MOTOR_STBY 24       // PC2 for TB6612FNG standby control
#define RIGHT_MOTOR_DIR1 25 // PC3 for right motor direction (AIN1)
#define RIGHT_MOTOR_DIR2 26 // PC4 for right motor direction (AIN2)
#define LEFT_MOTOR_MAX 255
#define RIGHT_MOTOR_MAX 255
#define TURN_MOTOR_OFFSET 20
#define EMITTER_PIN_PORT PORTC
#define EMITTER_PIN PC7
#define EMITTER_PIN_DDR DDRC
#define CALIBRATION_TIME_MS 5000 // 5 seconds for calibration
#define NUM_SENSORS 8
#define KP 0.02                   // Proportional gain for PID control
#define KD 0.05                   // Derivative gain for PID control
#define BASE_SPEED 70             // Base PWM value for motor speed (0-255)
#define LINE_POSITION_CENTER 3500 // Center position for 8 sensors (0–7000)
#define HALF_TURN_DELAY 470       // Turning 90 Degree delay
#define DELAY_BEFORE_TURN 310
#define FULL_TURN_DELAY 1100
#define BLACK_THRESHOLD 600
#define DEADBAND 500 // allow flexibility in error
#define JUNCTION_SCAN_DELAY 80
#define JUNCTION_SCAN_ITER 4

typedef struct
{
    uint16_t magic_number;    // Magic number to validate EEPROM data
    int16_t motor_offset;     // Motor speed offset
    int16_t base_speed;       // Base PWM value for motor speed
    double kp;                // Proportional gain for PID control
    double kd;                // Derivative gain 1 for PID control
    uint16_t black_threshold; // Threshold for detecting black line
    uint16_t turn_delay;      // Calculated delay for 90-degree turn (ms)
    uint16_t full_turn_delay; // Calculated delay for 90-degree turn (ms)
    uint16_t delay_before_turn;
    uint16_t deadband;
    int16_t junction_scan_delay;
    int16_t junction_scan_iter;
} EepromData;

// Calibration data structure
typedef struct
{
    uint16_t minValues[NUM_SENSORS];
    uint16_t maxValues[NUM_SENSORS];
} CalibrationData;

// Operating modes
enum Mode
{
    CMD,
    AUTONOMOUS,
    JUNCTION
};

// EEPROM variable declaration
volatile EEMEM EepromData ee_data = {
    .magic_number = EEPROM_MAGIC_NUMBER,
    .motor_offset = 4,
    .base_speed = BASE_SPEED,
    .kp = KP,
    .kd = KD,
    .black_threshold = BLACK_THRESHOLD,
    .turn_delay = HALF_TURN_DELAY,
    .full_turn_delay = FULL_TURN_DELAY,
    .delay_before_turn = DELAY_BEFORE_TURN,
    .deadband = DEADBAND,
    .junction_scan_delay = JUNCTION_SCAN_DELAY,
    .junction_scan_iter = JUNCTION_SCAN_ITER};
// Runtime copy of EEPROM data
volatile EepromData g_eeprom_data;
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

volatile int turnState = 0;
volatile int junctionState = 0;
volatile int junction_scan_count = 0;
void delay_ms(int16_t ms)
{
    while (ms--)
    {
        _delay_ms(1); // Delay 1 ms per iteration
    }
}
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
    g_leftSpeed = leftSpeed;
    g_rightSpeed = rightSpeed;

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
    }
    if (RIGHT_MOTOR_PWM == 18)
    {
        OCR1B = rightSpeed;
    }
}

void _subr_turnLeft()
{
    setMotorSpeeds(-g_eeprom_data.base_speed, g_eeprom_data.base_speed + g_eeprom_data.motor_offset);
    delay_ms(g_eeprom_data.turn_delay);
    setMotorSpeeds(0, 0);
}

void _subr_turnRight()
{

    setMotorSpeeds(g_eeprom_data.base_speed, -g_eeprom_data.base_speed - g_eeprom_data.motor_offset);
    delay_ms(g_eeprom_data.turn_delay);
    setMotorSpeeds(0, 0);
}

void _subr_goForward()
{
    setMotorSpeeds(g_eeprom_data.base_speed, g_eeprom_data.base_speed + g_eeprom_data.motor_offset);
    delay_ms(g_eeprom_data.turn_delay / 8);
    setMotorSpeeds(0, 0);
}

void _subr_goBack()
{
    setMotorSpeeds(-g_eeprom_data.base_speed, g_eeprom_data.base_speed + g_eeprom_data.motor_offset);
    delay_ms(g_eeprom_data.full_turn_delay);
    setMotorSpeeds(0, 0);
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
    // 1 to 7 for now
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    // Start conversion
    ADCSRA |= (1 << ADSC);
    // Wait for conversion to complete
    while (ADCSRA & (1 << ADSC))
        ;
    // Return 10-bit ADC value
    return ADC;
}

void QTR_EmitterOn(void)
{
    EMITTER_PIN_PORT |= (1 << EMITTER_PIN); // Set PD0 high
    _delay_us(100);                         // Wait for sensors to stabilize
}

void QTR_EmitterOff(void)
{
    EMITTER_PIN_PORT &= ~(1 << EMITTER_PIN); // Set PD0 low
    _delay_us(100);                          // Wait for sensors to stabilize
}

// Perform automated calibration by moving robot in zigzag pattern
void calibrateSensors(CalibrationData *calData)
{
    // Initialize min and max values
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        calData->minValues[i] = 1023; // Max ADC value
        calData->maxValues[i] = 0;    // Min ADC value
    }

    // Send calibration start message
    USART_SendString("Calibrating sensors... Move over line and surface\r\n");

    // Calibration loop for 5 seconds
    uint32_t startTime = 0;
    while (startTime < CALIBRATION_TIME_MS)
    {
        QTR_EmitterOn();
        for (uint8_t i = 0; i < NUM_SENSORS; i++)
        {
            uint16_t value = ADC_Read(i);
            if (value < calData->minValues[i])
                calData->minValues[i] = value;
            if (value > calData->maxValues[i])
                calData->maxValues[i] = value;
        }
        QTR_EmitterOff();
        delay_ms(20); // Sample every 20ms
        startTime += 20;
    }

    // Send calibration complete message with min/max values
    char buf[32];
    USART_SendString("Calibration complete. Min/Max values:\r\n");
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        snprintf(buf, sizeof(buf), "S%d: %d/%d ", i, calData->minValues[i], calData->maxValues[i]);
        USART_SendString(buf);
    }
    USART_SendString("\r\n");
}

void readCalibrated(uint16_t *sensorValues, uint16_t *calibratedValues, CalibrationData *calData)
{
    QTR_EmitterOn();
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        sensorValues[i] = ADC_Read(i);
        // Normalize to 0–1000
        if (calData->maxValues[i] > calData->minValues[i])
        {
            if (sensorValues[i] < calData->minValues[i])
            {
                calData->minValues[i] = sensorValues[i]; // in case sensor value becomes less than minimum calibrated value
            }
            calibratedValues[i] = ((uint32_t)(sensorValues[i] - calData->minValues[i]) * 1000) /
                                  (calData->maxValues[i] - calData->minValues[i]);
        }
        else
        {
            calibratedValues[i] = 0; // Avoid division by zero
        }
    }
    QTR_EmitterOff();
}

uint16_t readLinePosition(uint16_t *calibratedValues, uint8_t numSensors, uint8_t *blackSensors, uint8_t *minBlackSensorIdx, uint8_t *maxBlackSensorIdx)
{
    uint32_t avg = 0;
    uint16_t sum = 0;
    *blackSensors = 0;
    *minBlackSensorIdx = 8;
    *maxBlackSensorIdx = 0;
    for (uint8_t i = 0; i < numSensors; i++)
    {
        uint16_t value = calibratedValues[i];
        // Consider sensor "on line" if reading is above threshold (normalized scale)
        if (value > g_eeprom_data.black_threshold) // Threshold for normalized values (tune if needed)
        {
            avg += (uint32_t)value * (i * 1000);
            sum += value;
            *blackSensors = *blackSensors + 1;
            if (i < *minBlackSensorIdx)
                *minBlackSensorIdx = i;
            if (i > *maxBlackSensorIdx)
                *maxBlackSensorIdx = i;
        }
    }

    // If no line detected, return center position
    if (*blackSensors == 0)
    {
        return (numSensors - 1) * 500;
    }
    uint16_t result = (uint16_t)(avg / sum);

    // Calculate weighted average for line position
    return result;
}

void followLine(uint16_t *sensorValues, uint16_t *calibratedValues, CalibrationData *calData)
{
    static int16_t lastError = 0;
    static int16_t lastDError = 0;
    readCalibrated(sensorValues, calibratedValues, calData);
    // Count sensors detecting black
    uint8_t blackSensors;
    uint8_t minBlackSensorIdx;
    uint8_t maxBlackSensorIdx;

    if (junctionState == 1 || junctionState == -1)
    {
        junction_scan_count++;
        char buf[128];
        setMotorSpeeds(g_eeprom_data.base_speed * junctionState, (g_eeprom_data.base_speed + g_eeprom_data.motor_offset) * (-junctionState));
        delay_ms(g_eeprom_data.junction_scan_delay * junction_scan_count);
        setMotorSpeeds(0, 0);
        readLinePosition(calibratedValues, NUM_SENSORS, &blackSensors, &minBlackSensorIdx, &maxBlackSensorIdx);
        snprintf(buf, sizeof(buf), "blackSensors=%d, minIdx=%d, maxIdx=%d, l=%d, r=%d\r\n", blackSensors, minBlackSensorIdx, maxBlackSensorIdx, g_leftSpeed, g_rightSpeed);
        USART_SendString(buf);
        if (blackSensors >= 3)
        {
            snprintf(buf, sizeof(buf), "1.Line found, entered JUNCTION mode, blackSensor=%d l=%d, r=%d\r\n", blackSensors, g_leftSpeed, g_rightSpeed);
            USART_SendString(buf);
            setMotorSpeeds(g_eeprom_data.base_speed * (-junctionState), (g_eeprom_data.base_speed + g_eeprom_data.motor_offset) * junctionState);
            delay_ms(g_eeprom_data.junction_scan_delay * (junction_scan_count));
            setMotorSpeeds(0, 0);
            junctionState = 0;
            junction_scan_count = 0;
            robotMode = JUNCTION;
            USART_SendString("J\r\n");
            snprintf(buf, sizeof(buf), "2.Line found, entered JUNCTION mode, blackSensor=%d l=%d, r=%d\r\n", blackSensors, g_leftSpeed, g_rightSpeed);
            USART_SendString(buf);
            return;
        }
        if (junction_scan_count == g_eeprom_data.junction_scan_iter)
        {
            setMotorSpeeds(g_eeprom_data.base_speed * (-junctionState), (g_eeprom_data.base_speed + g_eeprom_data.motor_offset) * junctionState);
            delay_ms(g_eeprom_data.junction_scan_delay * (junction_scan_count / 2));
            junctionState = 0;
            junction_scan_count = 0;
            setMotorSpeeds(0, 0);
            robotMode = JUNCTION;
            USART_SendString("J\r\n");
            USART_SendString("Max scans reached, entered JUNCTION mode\r\n");
            return;
        }
        junctionState = -junctionState;
        return;
    }

    unsigned int position = readLinePosition(calibratedValues, NUM_SENSORS, &blackSensors, &minBlackSensorIdx, &maxBlackSensorIdx);

    char buf[32];
    USART_SendString("blackSensors ");
    snprintf(buf, sizeof(buf), "%d \r\n", blackSensors);
    USART_SendString(buf);
    char buff[32];
    USART_SendString("maxBlackSensorIdx, minBlackSensorIdx ");
    snprintf(buff, sizeof(buff), "%d, %d \r\n", maxBlackSensorIdx, minBlackSensorIdx);
    USART_SendString(buff);

    // turnState 0 : normal execution
    // turnState 1 : probably turn detected. go forward a bit and rotate until 4 or more IR sensor detect black line

    if (turnState == 1 && blackSensors > 3)
    {
        turnState = 0;
    }
    if (turnState == 1)
        return;
    // stop on white surface
    if (blackSensors == 0)
    {
        setMotorSpeeds(g_eeprom_data.base_speed, g_eeprom_data.base_speed + g_eeprom_data.motor_offset);
        delay_ms(g_eeprom_data.delay_before_turn);
        setMotorSpeeds(0, 0);
        readLinePosition(calibratedValues, NUM_SENSORS, &blackSensors, &minBlackSensorIdx, &maxBlackSensorIdx);
        char buff[32];
        USART_SendString("maxBlackSensorIdx, minBlackSensorIdx, blacksensors ");
        snprintf(buff, sizeof(buff), "%d, %d, %d \r\n", maxBlackSensorIdx, minBlackSensorIdx, blackSensors);
        USART_SendString(buff);
        if (blackSensors >= 2)
        {
            robotMode = JUNCTION;
            USART_SendString("J\r\n");
        }
        else
        {
            junction_scan_count = 0;
            junctionState = 1;
        }
        return;
    }
    // if (blackSensors != (maxBlackSensorIdx - minBlackSensorIdx + 1) || blackSensors <= 2)
    if (blackSensors != (maxBlackSensorIdx - minBlackSensorIdx + 1))
    {
        return;
    }

    if (blackSensors > 4 && (calibratedValues[0] > g_eeprom_data.deadband || calibratedValues[NUM_SENSORS - 1] > g_eeprom_data.deadband))
    {
        setMotorSpeeds(g_eeprom_data.base_speed, g_eeprom_data.base_speed + g_eeprom_data.motor_offset);
        delay_ms(g_eeprom_data.delay_before_turn);
        int sign = calibratedValues[0] > g_eeprom_data.deadband ? -1 : 1;
        setMotorSpeeds((g_eeprom_data.base_speed) * sign, (g_eeprom_data.base_speed + g_eeprom_data.motor_offset) * (-sign));
        turnState = 1;
        return;
    }

    int16_t error = position - LINE_POSITION_CENTER;
    int16_t correction;
    int16_t dError = 0;

    // allow small error for minimizing jerking
    if (abs(error) < g_eeprom_data.deadband)
    {
        correction = 0;
    }
    else
    {
        dError = error - lastError;
        dError = dError * 3 / 4 + lastDError * 1 / 4;
        lastDError = dError;
        correction = g_eeprom_data.kp * error + g_eeprom_data.kd * dError;
    }
    lastError = error;
    int16_t leftSpeed = g_eeprom_data.base_speed + correction;
    int16_t rightSpeed = g_eeprom_data.base_speed + g_eeprom_data.motor_offset - correction;
    setMotorSpeeds(leftSpeed, rightSpeed);
}

const char *getDirection(uint16_t *calibratedValues, uint8_t numSensors)
{
    uint16_t leftSum = 0, centerSum = 0, rightSum = 0;

    // Sum calibrated values for each region
    for (uint8_t i = 0; i < numSensors; i++)
    {
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

void _cmd_right()
{
    setMotorSpeeds(g_leftSpeed, g_rightSpeed + TURN_MOTOR_OFFSET);
    delay_ms(g_eeprom_data.turn_delay);
    setMotorSpeeds(g_leftSpeed, g_rightSpeed);
}
void _cmd_left()
{
    setMotorSpeeds(g_leftSpeed + TURN_MOTOR_OFFSET, g_rightSpeed);
    delay_ms(g_eeprom_data.turn_delay);
    setMotorSpeeds(g_leftSpeed, g_rightSpeed);
}
void _cmd_forward()
{
    setMotorSpeeds(BASE_SPEED, BASE_SPEED + g_eeprom_data.motor_offset);
}
void _cmd_slower()
{
    setMotorSpeeds(g_leftSpeed - 10, g_rightSpeed - 10);
}
void _cmd_break()
{
    setMotorSpeeds(0, 0);
}
// EEPROM management functions
void eeprom_write_data(const EepromData *data)
{
    uint8_t sreg = SREG;
    cli();
    eeprom_update_block(data, (void *)&ee_data, sizeof(EepromData));
    SREG = sreg;
}

void eeprom_read_data(EepromData *data)
{
    eeprom_read_block(data, (const void *)&ee_data, sizeof(EepromData));
}

void eeprom_init()
{
    EepromData temp;
    eeprom_read_data(&temp);
    if (temp.magic_number != EEPROM_MAGIC_NUMBER)
    {
        // EEPROM uninitialized, load defaults
        temp.magic_number = EEPROM_MAGIC_NUMBER;
        temp.motor_offset = 4;
        temp.base_speed = BASE_SPEED;
        temp.kp = KP;
        temp.kd = KD;
        temp.black_threshold = BLACK_THRESHOLD;
        temp.turn_delay = HALF_TURN_DELAY;
        temp.full_turn_delay = FULL_TURN_DELAY;
        temp.delay_before_turn = DELAY_BEFORE_TURN;
        temp.deadband = DEADBAND;
        temp.junction_scan_delay = JUNCTION_SCAN_DELAY;
        temp.junction_scan_iter = JUNCTION_SCAN_ITER;
        eeprom_write_data(&temp);
    }
    memcpy((void *)&g_eeprom_data, (void *)&temp, sizeof(EepromData));
}
void _cmd_eeprom_default()
{
    EepromData temp;
    // EEPROM uninitialized, load defaults
    temp.magic_number = EEPROM_MAGIC_NUMBER;
    temp.motor_offset = 4;
    temp.base_speed = BASE_SPEED;
    temp.kp = KP;
    temp.kd = KD;
    temp.black_threshold = BLACK_THRESHOLD;
    temp.turn_delay = HALF_TURN_DELAY;
    temp.full_turn_delay = FULL_TURN_DELAY;
    temp.delay_before_turn = DELAY_BEFORE_TURN;
    temp.deadband = DEADBAND;
    temp.junction_scan_delay = JUNCTION_SCAN_DELAY;
    temp.junction_scan_iter = JUNCTION_SCAN_ITER;
    eeprom_write_data(&temp);
    memcpy((void *)&g_eeprom_data, (void *)&temp, sizeof(EepromData));
}
void _cmd_increase_motor_offset()
{
    g_eeprom_data.motor_offset += 2;
    eeprom_write_data((const EepromData *)&g_eeprom_data);
    char buf[32];
    snprintf(buf, sizeof(buf), "motor offset increased to %d\r\n", g_eeprom_data.motor_offset);
    USART_SendString(buf);
}

void _cmd_decrease_motor_offset()
{
    g_eeprom_data.motor_offset -= 2;
    eeprom_write_data((const EepromData *)&g_eeprom_data);
    char buf[32];
    snprintf(buf, sizeof(buf), "motor offset decreased to %d\r\n", g_eeprom_data.motor_offset);
    USART_SendString(buf);
}

void _cmd_set_base_speed(int16_t speed)
{
    g_eeprom_data.base_speed = speed > LEFT_MOTOR_MAX ? LEFT_MOTOR_MAX : speed;
    eeprom_write_data((const EepromData *)&g_eeprom_data);
    char buf[32];
    snprintf(buf, sizeof(buf), "Base speed set to %d\r\n", g_eeprom_data.base_speed);
    USART_SendString(buf);
}

void _cmd_set_kp(double kp)
{
    g_eeprom_data.kp = kp;
    eeprom_write_data((const EepromData *)&g_eeprom_data);
    char buf[32];
    snprintf(buf, sizeof(buf), "KP set to %.3f\r\n", g_eeprom_data.kp);
    USART_SendString(buf);
}

void _cmd_set_kd(double kd)
{
    g_eeprom_data.kd = kd;
    eeprom_write_data((const EepromData *)&g_eeprom_data);
    char buf[32];
    snprintf(buf, sizeof(buf), "KD set to %.3f\r\n", g_eeprom_data.kd);
    USART_SendString(buf);
}
void _cmd_set_black_threshold(uint16_t val)
{
    g_eeprom_data.black_threshold = val;
    eeprom_write_data((const EepromData *)&g_eeprom_data);
    char buf[32];
    snprintf(buf, sizeof(buf), "Black Threshold set to %d\r\n", g_eeprom_data.black_threshold);
    USART_SendString(buf);
}
void _cmd_set_turn_delay(uint16_t delay)
{
    g_eeprom_data.turn_delay = delay > 0 ? delay : 0;
    eeprom_write_data((const EepromData *)&g_eeprom_data);
    char buf[32];
    snprintf(buf, sizeof(buf), "Turn Delay set to %d\r\n", g_eeprom_data.turn_delay);
    USART_SendString(buf);
}
void _cmd_set_full_turn_delay(uint16_t delay)
{
    g_eeprom_data.full_turn_delay = delay > 0 ? delay : 0;
    eeprom_write_data((const EepromData *)&g_eeprom_data);
    char buf[32];
    snprintf(buf, sizeof(buf), "Full Turn Delay set to %d\r\n", g_eeprom_data.full_turn_delay);
    USART_SendString(buf);
}
void _cmd_set_delay_before_turn(uint16_t delay)
{
    g_eeprom_data.delay_before_turn = delay > 0 ? delay : 0;
    eeprom_write_data((const EepromData *)&g_eeprom_data);
    char buf[32];
    snprintf(buf, sizeof(buf), "Delay Before Turn set to %d\r\n", g_eeprom_data.delay_before_turn);
    USART_SendString(buf);
}
void _cmd_set_junction_scan_delay(int16_t delay)
{
    g_eeprom_data.junction_scan_delay = delay;
    eeprom_write_data((const EepromData *)&g_eeprom_data);
    char buf[32];
    snprintf(buf, sizeof(buf), "Junction Scan Delay Count set to %d\r\n", g_eeprom_data.junction_scan_delay);
    USART_SendString(buf);
}

void _cmd_set_dead_band(uint16_t val)
{
    g_eeprom_data.deadband = val;
    eeprom_write_data((const EepromData *)&g_eeprom_data);
    char buf[32];
    snprintf(buf, sizeof(buf), "Dead Band set to %d\r\n", g_eeprom_data.deadband);
    USART_SendString(buf);
}
void _cmd_set_junction_scan_iter(int16_t val)
{
    g_eeprom_data.junction_scan_iter = val;
    eeprom_write_data((const EepromData *)&g_eeprom_data);
    char buf[32];
    snprintf(buf, sizeof(buf), "Junction Scan Iter set to %d\r\n", g_eeprom_data.junction_scan_iter);
    USART_SendString(buf);
}

void _cmd_showSensorVal(uint16_t *sensorValues, uint16_t *calibratedValues, CalibrationData calData)
{
    readCalibrated(sensorValues, calibratedValues, &calData);
    char buf[64];
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        snprintf(buf, sizeof(buf), "calMax: %d, calMin: %d, sen: %d, cal: %d\r\n", calData.maxValues[i], calData.minValues[i], sensorValues[i], calibratedValues[i]);
        USART_SendString(buf);
    }
    const char *direction = getDirection(calibratedValues, NUM_SENSORS);
    snprintf(buf, sizeof(buf), "Dir: %s\r\n", direction);
    USART_SendString(buf);
    char buff[128];
    snprintf(buff, sizeof(buff), "mo: %d, bs: %d, kp: %d, kd: %d, bt: %d, td: %d, ftd: %d, db: %d, sd: %d, db: %d, si: %d\r\n", g_eeprom_data.motor_offset, g_eeprom_data.base_speed, (int)(g_eeprom_data.kp * 1000), (int)(g_eeprom_data.kd * 1000), g_eeprom_data.black_threshold, g_eeprom_data.turn_delay, g_eeprom_data.full_turn_delay, g_eeprom_data.delay_before_turn, g_eeprom_data.junction_scan_delay, g_eeprom_data.deadband, g_eeprom_data.junction_scan_iter);
    USART_SendString(buff);
}

int main(void)
{
    USART_Init(); /* initialize USART with 115200 baud rate */
    sei();        /* enable global interrupts for USART RX */
    initPWM();
    ADC_Init();
    MCUCSR |= (1 << JTD);                  // Disable JTAG
    MCUCSR |= (1 << JTD);                  // Write twice in the same cycle
    EMITTER_PIN_DDR |= (1 << EMITTER_PIN); // Set emitter pin output
    CalibrationData calData;
    uint16_t sensorValues[NUM_SENSORS];
    uint16_t calibratedValues[NUM_SENSORS];
    setMotorSpeeds(0, 0);
    eeprom_init();
    while (1)
    {
        /* Check if a string is ready */
        if (USART_IsStringReady())
        {
            char *received = USART_GetStringBuffer();
            char cmd = received[0];
            if (robotMode == CMD)
            {
                switch (cmd)
                {
                case 'R':
                    _cmd_right();
                    break;
                case 'L':
                    _cmd_left();
                    break;
                case 'F':
                    _cmd_forward();
                    break;
                case 'S':
                    _cmd_slower();
                    break;
                case 'B':
                    _cmd_break();
                    break;
                case 'C':
                    calibrateSensors(&calData);
                    break;
                case 'K':
                    _cmd_showSensorVal(sensorValues, calibratedValues, calData);
                    break;
                case 'I':
                    _cmd_increase_motor_offset();
                    break;
                case 'D':
                    _cmd_decrease_motor_offset();
                    break;
                case 'E':
                    _cmd_eeprom_default();
                    break;
                case 'P':
                {
                    double kp = atof(&received[1]);
                    _cmd_set_kp(kp);
                    break;
                }
                case 'Q':
                {
                    double kd = atof(&received[1]);
                    _cmd_set_kd(kd);
                    break;
                }
                case 'V':
                {
                    int16_t speed = atoi(&received[1]);
                    _cmd_set_base_speed(speed);
                    break;
                }
                case 'W':
                {
                    uint16_t val = atoi(&received[1]);
                    _cmd_set_black_threshold(val);
                    break;
                }
                case 'X':
                {
                    int16_t delay = atoi(&received[1]);
                    _cmd_set_turn_delay(delay);
                    break;
                }
                case 'Y':
                {
                    int16_t delay = atoi(&received[1]);
                    _cmd_set_delay_before_turn(delay);
                    break;
                }
                case 'Z':
                {
                    int16_t delay = atoi(&received[1]);
                    _cmd_set_junction_scan_delay(delay);
                    break;
                }
                case 'O':
                {
                    int16_t val = atoi(&received[1]);
                    _cmd_set_junction_scan_iter(val);
                    break;
                }
                case 'U':
                {
                    int16_t val = atoi(&received[1]);
                    _cmd_set_dead_band(val);
                    break;
                }
                case 'J':
                {
                    int16_t val = atoi(&received[1]);
                    _cmd_set_full_turn_delay(val);
                    break;
                }
                case 'T':
                    USART_SendString("Switching to AUTONOMOUS mode");
                    USART_SendString("\r\n");
                    robotMode = AUTONOMOUS;

                    break;
                }
            }
            else if (robotMode == JUNCTION)
            {
                switch (cmd)
                {
                case 'R':
                    _subr_turnRight();

                    robotMode = AUTONOMOUS;

                    break;
                case 'F':
                    _subr_goForward();
                    robotMode = AUTONOMOUS;

                    break;
                case 'L':
                    _subr_turnLeft();
                    robotMode = AUTONOMOUS;

                    break;
                case 'B':
                    _subr_goBack();
                    robotMode = AUTONOMOUS;

                    break;
                case 'T':
                    USART_SendString("Switching to CMD mode");
                    USART_SendString("\r\n");
                    robotMode = CMD;
                    break;
                }
            }
            else if (robotMode == AUTONOMOUS)
            {
                switch (cmd)
                {
                case 'T':
                    USART_SendString("Switching to CMD mode");
                    USART_SendString("\r\n");
                    setMotorSpeeds(0, 0);
                    robotMode = CMD;
                    break;
                }
            }
            USART_ClearStringBuffer();
            USART_SetStringReady(0);
        }
        if (robotMode == AUTONOMOUS)
        {
            followLine(sensorValues, calibratedValues, &calData);
        }
    }
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
}