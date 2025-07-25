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
volatile uint16_t EEMEM ee_motor_offset;

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
#define LED PORTB /* connected LED on PORT pin */
#define EMITTER_PIN_PORT PORTC
#define EMITTER_PIN PC7
#define EMITTER_PIN_DDR DDRC
#define CALIBRATION_TIME_MS 5000 // 5 seconds for calibration
#define NUM_SENSORS 7
// PID constants
#define KP 0.02                   // Proportional gain for PID control
#define KD_1 0.05                 // Derivative gain for PID control
#define KD_2 1.5                  // Derivative gain for PID control
#define BASE_SPEED 70             // Base PWM value for motor speed (0-255)
#define LINE_POSITION_CENTER 3000 // Center position for 8 sensors (0–7000)
#define NO_SURFACE_THRESHOLD 1000 // Threshold for detecting no surface
#define HALF_TURN_DELAY 1000      // Turning 90 Degree delay
#define DELAY_BEFORE_TURN 100
#define DELAY_AFTER_TURN 300
#define MAX_CORRECTION 100
#define TURN_DERROR_THRESHOLD 1500 // when turning, derror increases significantly. In this case, turning velocity should be adjusted (increased) so that IR stay on line. In this case, use KD_2
#define DEADBAND 1000              // allow flexibility in error
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
volatile int16_t motor_offset;
volatile uint16_t black_threshold = NO_SURFACE_THRESHOLD; // default threshold for detecting black line

volatile int state = 0;
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
    setMotorSpeeds(BASE_SPEED, BASE_SPEED + motor_offset);
    _delay_ms(DELAY_BEFORE_TURN);
    setMotorSpeeds(0, BASE_SPEED);
    _delay_ms(HALF_TURN_DELAY);
    setMotorSpeeds(BASE_SPEED, BASE_SPEED + motor_offset);
    _delay_ms(DELAY_AFTER_TURN);
    setMotorSpeeds(0, 0);
}
void _subr_turnRight()
{
    setMotorSpeeds(BASE_SPEED, BASE_SPEED + motor_offset);
    _delay_ms(DELAY_BEFORE_TURN);
    setMotorSpeeds(BASE_SPEED, 0);
    _delay_ms(HALF_TURN_DELAY);
    setMotorSpeeds(BASE_SPEED, BASE_SPEED + motor_offset);
    _delay_ms(DELAY_AFTER_TURN);
    setMotorSpeeds(0, 0);
}
void _subr_goForward()
{
    setMotorSpeeds(BASE_SPEED, BASE_SPEED + motor_offset);
    _delay_ms(1000);
    setMotorSpeeds(0, 0);
}
void _subr_goBack()
{
    setMotorSpeeds(0, BASE_SPEED);
    _delay_ms(HALF_TURN_DELAY * 2);
    setMotorSpeeds(BASE_SPEED, BASE_SPEED + motor_offset);
    _delay_ms(DELAY_AFTER_TURN);
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
    ADMUX = (ADMUX & 0xF0) | ((channel + 1) & 0x0F);
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
        _delay_ms(20); // Sample every 20ms
        startTime += 20;
    }
    // set black threshold
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        if (black_threshold > calData->maxValues[i])
            black_threshold = calData->maxValues[i];
    }
    black_threshold -= 50; // make the threshold a bit flexible
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
        if (value > 500) // Threshold for normalized values (tune if needed)
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
    uint16_t result = (uint16_t)(avg / sum);

    if (*blackSensors > 5)
    {
        if (result > LINE_POSITION_CENTER)
        {

            return 4000;
        }
        else
        {
            return 2000;
        }
    }

    // If no line detected, return center position
    if (*blackSensors == 0)
    {
        return (numSensors - 1) * 500;
    }

    // Calculate weighted average for line position
    return result;
}

void followLine(uint16_t *sensorValues, uint16_t *calibratedValues, CalibrationData *calData)
{
    static int16_t lastError = 0;
    static int16_t lastDError = 0;
    readCalibrated(sensorValues, calibratedValues, calData);
    // Logic based on number of sensors detecting black
    // if (blackSensors >= 8)
    // {
    //     setMotorSpeeds(0, 0); // Stop robot
    //     robotMode = CMD;
    //     USART_SendString("Victory\r\n");
    // }
    // else
    // if (blackSensors ==0)
    // {
    //         // if the line is junction given that avg of max and min black sensor idx is towards mid
    //         setMotorSpeeds(0, 0); // Stop robot
    //         robotMode = CMD;
    //         USART_SendString("Junction\r\n");
    // }
    // else {
    // when at least 2 sensor senses black, keeping the detection at reasonable width for avoiding noise
    // Count sensors detecting black
    uint8_t blackSensors;
    uint8_t minBlackSensorIdx;
    uint8_t maxBlackSensorIdx;
    unsigned int position = readLinePosition(calibratedValues, NUM_SENSORS, &blackSensors, &minBlackSensorIdx, &maxBlackSensorIdx);
    char buf[32];
    USART_SendString("blackSensors ");
    snprintf(buf, sizeof(buf), "%d \r\n", blackSensors);
    USART_SendString(buf);
    char buff[32];
    USART_SendString("maxBlackSensorIdx, minBlackSensorIdx ");
    snprintf(buff, sizeof(buff), "%d, %d \r\n", maxBlackSensorIdx, minBlackSensorIdx);
    USART_SendString(buff);
    // stop on white surface
    if(state == 3) {
        if(blackSensors > 0){
            state = 0;
        }else{
            return;
        }

    }
    if(state == 1) {
        if(blackSensors == 0){
            state = 2;
        }else{
            return;
        }
    }
    if (state != 2 && blackSensors == 0)
    {
        setMotorSpeeds(0, 0); // Stop robot
        robotMode = CMD;
        USART_SendString("Junction\r\n");
        return;
    }

    int16_t error = position - LINE_POSITION_CENTER;
    int16_t correction;
    int16_t dError = 0;
    if (abs(error) < DEADBAND)
    {
        correction = 0;
    }
    else
    {
        dError = error - lastError;
        dError = dError * 3 / 4 + lastDError * 1 / 4;
        lastDError = dError;
        // if (abs(dError) < TURN_DERROR_THRESHOLD)
        // {
            correction = KP * error + KD_1 * dError;
        // }
        // else
        // {
        //     correction = 0.07 * error + KD_2 * dError;
        // }
    }
    lastError = error;
    int16_t leftSpeed = BASE_SPEED + correction;
    int16_t rightSpeed = BASE_SPEED + motor_offset - correction;
    char buffe[64];
    USART_SendString("error, lastDerror, correction, leftspeed, rightspeed \r\n");
    snprintf(buffe, sizeof(buffe), "%d, %d, %d, %d, %d \r\n", error, lastDError, correction, leftSpeed, rightSpeed);
    USART_SendString(buffe);
    if(blackSensors > 5){
        if(state == 0){
            setMotorSpeeds(-rightSpeed, -leftSpeed);
            state = 1;
        }
        if(state == 2 ){
            setMotorSpeeds(BASE_SPEED, BASE_SPEED+motor_offset);
            // _delay_ms(400);
            state = 3;
        }
    }else{
        setMotorSpeeds(leftSpeed, rightSpeed);
    }
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
    _delay_ms(HALF_TURN_DELAY);
    setMotorSpeeds(g_leftSpeed, g_rightSpeed);
}
void _cmd_left()
{
    setMotorSpeeds(g_leftSpeed + TURN_MOTOR_OFFSET, g_rightSpeed);
    _delay_ms(HALF_TURN_DELAY);
    setMotorSpeeds(g_leftSpeed, g_rightSpeed);
}
void _cmd_forward()
{
    setMotorSpeeds(BASE_SPEED, BASE_SPEED + motor_offset);
}
void _cmd_slower()
{
    setMotorSpeeds(g_leftSpeed - 10, g_rightSpeed - 10);
}
void _cmd_break()
{
    setMotorSpeeds(0, 0);
}
void save_motor_offset(int16_t v)
{
    /* Disable interrupts briefly so an ISR cannot interrupt
       us between erase and write.  Not strictly needed if no
       ISR touches EEPROM, but it’s good practice.             */
    uint8_t sreg = SREG;
    cli();

    eeprom_update_word((uint16_t *)&ee_motor_offset, v); /* avr-libc call */

    SREG = sreg; /* restore interrupt state */
}
int16_t load_motor_offset(void)
{
    return (int16_t)eeprom_read_word((uint16_t *)&ee_motor_offset);
}
void _cmd_increase_motor_offset()
{
    motor_offset += 2;
    save_motor_offset(motor_offset);
    char buf[32];
    snprintf(buf, sizeof(buf), "motor offset increased to %d\r\n", motor_offset);
    USART_SendString(buf);
}
void _cmd_decrease_motor_offset()
{
    motor_offset -= 2;
    save_motor_offset(motor_offset);
    char buf[32];
    snprintf(buf, sizeof(buf), "motor offset decreased to %d\r\n", motor_offset);
    USART_SendString(buf);
}
void _cmd_showSensorVal(uint16_t *sensorValues, uint16_t *calibratedValues, CalibrationData calData)
{
    readCalibrated(sensorValues, calibratedValues, &calData);
    char buf[64];
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        // snprintf(buf, sizeof(buf), "sensorVal: %d\r\n", sensorValues[i]);
        // USART_SendString(buf);
        snprintf(buf, sizeof(buf), "calMax: %d, calMin: %d, sen: %d, cal: %d\r\n", calData.maxValues[i], calData.minValues[i], sensorValues[i], calibratedValues[i]);
        USART_SendString(buf);
    }
    const char *direction = getDirection(calibratedValues, NUM_SENSORS);
    snprintf(buf, sizeof(buf), "Dir: %s\r\n", direction);
    USART_SendString(buf);
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
    motor_offset = load_motor_offset();
    if (motor_offset == 0xFFFF)
    {
        motor_offset = 0;
        save_motor_offset(motor_offset);
    }
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
                case 'T':
                    USART_SendString("Switching to AUTONOMOUS mode");
                    USART_SendString("\r\n");
                    robotMode = AUTONOMOUS;
                    break;
                }
            }
            else
            {
                switch (cmd)
                {
                case 'R':
                    _subr_turnRight();
                    break;
                case 'F':
                    _subr_goForward();
                    break;
                case 'L':
                    _subr_turnLeft();
                    break;
                case 'B':
                    _subr_goBack();
                    break;
                case 'T':
                    USART_SendString("Switching to CMD mode");
                    USART_SendString("\r\n");
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