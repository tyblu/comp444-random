/*
* SonarSensor.cpp
*
*  Created on: Oct 26, 2017
*      Author: Tyler Lucas <tyblu@live.com>
*/

#include "SonarSensor.h"

#define MAX_ECHO_TIME 2915L	// 2 * 0.5 meters / speed of sound = 2915.4519 uS

#define log, logComma(arg_file);

//#define TybluLsq_DEBUG_MODE
#ifdef TybluLsq_DEBUG_MODE
#	include <Arduino.h>	// only for Serial debug messages
#	define DEBUG1(x) Serial.print("TybluLsq : "); Serial.println(x); delay(2)	// note missing ';'
#	define DEBUG2(x,y) Serial.print("TybluLsq : "); Serial.print(x); Serial.println(y); delay(2)	// note missing ';'
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
long SonarSensor::getMeasurement()
{
	digitalWrite(triggerPin, LOW);
	delayMicroseconds(2);
	digitalWrite(triggerPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(triggerPin, LOW);

	return pulseIn(echoPin, HIGH, MAX_ECHO_TIME);
}

