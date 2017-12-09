/*
* SonarSensor.cpp
*
*  Created on: Oct 26, 2017
*      Author: Tyler Lucas <tyblu@live.com>
*/

#include "SonarSensor.h"

#define MAX_ECHO_TIME_US 2332UL	// 2 * 40 cm / speed of sound in microseconds = 2332...
#define MEASUREMENT_DELAY_MS 60	// time between measurements [ms]

//#define TybluLsq_DEBUG_MODE
#ifdef TybluLsq_DEBUG_MODE
#	include <Arduino.h>	// only for Serial debug messages
#	define DEBUG1(x) Serial.print("SonarSensor : "); Serial.println(x); delay(2)	// note missing ';'
#	define DEBUG2(x,y) Serial.print("SonarSensor : "); Serial.print(x); Serial.println(y); delay(2)	// note missing ';'
#else
#	define DEBUG1(x)
#	define DEBUG2(x,y)
#endif

/* Post-conditions:
 - triggerPin set to output and LOW
 - echoPin set to input
 */
SonarSensor::SonarSensor(int triggerPin, int echoPin)
	: triggerPin(triggerPin)
	, echoPin(echoPin)
{
	pinMode(triggerPin, OUTPUT);
	digitalWrite(triggerPin, LOW);
	pinMode(echoPin, INPUT);
}

/* Preconditions: triggerPin is LOW. */
uint32_t SonarSensor::getMeasurement()
{
	digitalWrite(triggerPin, LOW);
	delayMicroseconds(2);
	digitalWrite(triggerPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(triggerPin, LOW);

	return pulseIn(echoPin, HIGH, MAX_ECHO_TIME_US);
}

uint32_t SonarSensor::getMeasurement(uint16_t count)
{
	uint32_t sum = 0;
	uint16_t iteration = 0;
	while (iteration < count)
	{
		sum += getMeasurement();
		delay(MEASUREMENT_DELAY_MS);
		iteration++;
	}
	return (sum + count/2)/ count;
}