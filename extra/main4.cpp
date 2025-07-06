#include <avr/io.h>
#include <util/delay.h>
#include <PololuQTRSensors.h>
#include <USART_RS232_H_file.h>
#include <stdio.h>

// QTR-8A sensor pins
#define EMITTER_PIN 29   // PC7 for QTR sensor emitter control

int main(void)
{
    // Initialize USART for debugging
    USART_Init();
    sei();                                    // Enable global interrupts
    USART_SendString("ATmega32A QTR Test Started\r\n");

    // Initialize QTR-8A sensor
    QTRSensors qtr;
    qtr.setTypeAnalog();                      // Configure for analog sensors
    uint8_t sensorPins[] = {40, 39, 38, 37, 36, 35, 34, 33}; // PA0-PA7 (pins 40-33)
    qtr.setSensorPins(sensorPins, 8);         // Set sensor pins
    qtr.setEmitterPin(EMITTER_PIN);           // Set emitter control pin

    // Perform calibration (move sensor over line manually during this period)
    // for (uint8_t i = 0; i < 50; i++) {
    //     qtr.calibrate(QTRReadMode::On); // Calibrate with emitters on and off
    //     _delay_ms(20);                        // Delay between readings
    // }
    USART_SendString("Calibration Complete\r\n");

    uint16_t sensorValues[8];                 // Array to store sensor readings

    while (1) {
        // Read sensor values
        qtr.read(sensorValues, QTRReadMode::Manual);

        // Send sensor values via USART
        USART_SendString("Values: ");
        for (uint8_t i = 0; i < 8; i++) {
            char buf[6];
            snprintf(buf, sizeof(buf), "%d ", sensorValues[i]);
            USART_SendString(buf);
        }
        USART_SendString("\r\n");

        _delay_ms(1000);                       // Delay between readings
        USART_SendString("Status: OK\r\n");    // Confirm program is running
    }

    return 0;                                 // Unreachable, included for completeness
}