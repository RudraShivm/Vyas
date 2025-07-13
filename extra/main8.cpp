#include <avr/io.h>
#include <param.h>
#include <string.h>
#include <stdio.h>
#include <PololuQTRSensors.h>
#include "USART_RS232_H_file.h"
#include <util/delay.h>
#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define EMITTER_PIN_PORT PORTC
#define EMITTER_PIN PC7
#define EMITTER_PIN_DDR DDRC
#define CALIBRATION_TIME_MS 5000 // 5 seconds for calibration
#define NUM_SENSORS 8

// Calibration data structure
typedef struct {
    uint16_t minValues[NUM_SENSORS];
    uint16_t maxValues[NUM_SENSORS];
} CalibrationData;

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
    // Initialize UART for Bluetooth
    USART_Init();
    sei();
    // Disable JTAG to free up PC2 and PC3
    MCUCSR |= (1 << JTD);
    MCUCSR |= (1 << JTD);

    // Initialize ADC
    ADC_Init();

    // Set emitter pin output (PD0, pin 14)
    EMITTER_PIN_DDR |= (1 << EMITTER_PIN);

    CalibrationData calData;
    uint16_t sensorValues[NUM_SENSORS];
    uint16_t calibratedValues[NUM_SENSORS];
    char buf[32];

    // Send startup message
    USART_SendString("QTR-8A with Calibration, Line Position, and Direction\r\n");

    // Perform calibration
    calibrateSensors(&calData);

    while (1)
    {
        // Read and calibrate sensor values
        readCalibrated(sensorValues, calibratedValues, &calData);

        // Calculate line position using calibrated values
        uint16_t position = readLinePosition(calibratedValues, NUM_SENSORS);

        // Get direction based on calibrated values
        const char* direction = getDirection(calibratedValues, NUM_SENSORS);

        // Send raw and calibrated sensor values over UART
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
            snprintf(buf, sizeof(buf), "S%d:%d/%d ", i, sensorValues[i], calibratedValues[i]);
            USART_SendString(buf);
        }

        // Send line position and direction
        snprintf(buf, sizeof(buf), "Pos: %d, Dir: %s\r\n", position, direction);
        USART_SendString(buf);

        _delay_ms(500); // Update every 500ms
    }

    return 0;
}