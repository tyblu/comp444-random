/*
 * AutoMovePinDefinitions.h
 *
 *  Created on: Nov 2, 2017
 *      Author: Tyler Lucas
 */

#ifndef AutoMovePinDefinitions_h
#define AutoMovePinDefinitions_h

 // Servo stuff.
#define BOOM1_PWM_PIN 6
#define BOOM1_ADC_PIN A1
#define BOOM2_PWM_PIN 9
#define BOOM2_ADC_PIN A0
#define TURRET_PWM_PIN 3
#define TURRET_ADC_PIN A3	// currently unused
#define CLAW_PWM_PIN 5
#define CLAW_ADC_PIN A2
#define SERVO_POWER_CONTROL_PIN 8
#define SERVO_POWER_FEEDBACK_PIN 1	// disconnected, interferes with USB TX/RX

 // Sonar stuff.
#define SONAR_TRIGGER_PIN 4
#define SONAR_ECHO_PIN 7

 // SPI SD card stuff.
#define SD_CS_PIN 10	// hardware SS pin
#define SD_SCK_PIN 13	// ICSP #3
#define SD_MOSI_PIN 11	// ICSP #4
#define SD_MISO_PIN 12	// ICSP #1

 // Force sensor stuff.
#define FORCE_SENSOR_ANALOG_A_PIN A4
#define FORCE_SENSOR_ANALOG_B_PIN A5
#define FORCE_SENSORS_POWER_PIN 2

#endif	// AutoMovePinDefinitions_h