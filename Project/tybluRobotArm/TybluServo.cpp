/*
 * TybluServo.cpp
 *
 *  Created on: Oct 14, 2017
 *      Author: tyblu
 */

#include "TybluServo.h"
#include "QuickStats.h"

#define MODE_EPSILON 1.0
#define MODE_LOWER_LIMIT 10		// 5V *  10/1024 = 49mV
#define MODE_UPPER_LIMIT 512	// 5V * 512/1024 = 2.5V, max in equal voltage divider

TybluServo::TybluServo() : Servo()
{
	this->minAngle = 0;
	this->maxAngle = 180;
	this->sensorPin = -1;
	this->measurementsCount = 1;
	this->sensorSlope = 1.0;
	this->sensorOffset = 0.0;
}

TybluServo::TybluServo(int min, int max, int sensorPin,
		float sensorSlope, float sensorOffset, int pt_count) : Servo()
{
	this->minAngle = min;
	this->maxAngle = max;
	this->sensorPin = sensorPin;
	this->measurementsCount = pt_count;
	this->sensorSlope = sensorSlope;
	this->sensorOffset = sensorOffset;
}

void TybluServo::setMinAngle(int minAngle)
{
	this->minAngle = minAngle;
}

void TybluServo::setMaxAngle(int maxAngle)
{
	this->maxAngle = maxAngle;
}

void TybluServo::setSensorPin(int sensorPin)
{
	this->sensorPin = sensorPin;
}

void TybluServo::setMeasurementsCount(int pt_count)
{
	this->measurementsCount = pt_count;
}

void TybluServo::setSensorConstants(float slope, float offset)
{
	this->sensorSlope = slope;
	this->sensorOffset = offset;
}

int TybluServo::getMinAngle()
{
	return this->minAngle;
}

int TybluServo::getMaxAngle()
{
	return this->maxAngle;
}

int TybluServo::getAnalogAngle()
{
	float measurements[measurementsCount];
	float stDev = analogDeviationLimit + 1.0;

	while ( stDev > analogDeviationLimit)
	{
		for (int i=0; i<measurementsCount; i++)
			measurements[i] = analogRead(sensorPin);
		qs.bubbleSort(measurements, measurementsCount);
		stDev = qs.stdev(measurements, measurementsCount);
//		Serial.print(" STDEV=");
//		Serial.println(stDev);
	}

	float mode = qs.mode(measurements, measurementsCount, MODE_EPSILON);
//	Serial.print(" MODE=");
//	Serial.print(mode);
//	Serial.print(" SLOPE=");
//	Serial.print( sensorSlope );
//	Serial.print(" OFFSET=");
//	Serial.print( sensorOffset );
//	Serial.write(' ');

	if (mode < MODE_LOWER_LIMIT || mode > MODE_UPPER_LIMIT)
		return 0;
	else
		return (int)mode * sensorSlope + sensorOffset;
}

/**
 * If value is < 200 its treated as an angle, otherwise as pulse width in
 * microseconds. If value is < minAngle then it's set to minAngle; if > maxAngle and
 * < 200 then it's set to maxAngle.
 */
void TybluServo::write(int value)
{
	if (value < minAngle)
		Servo::write(minAngle);
	else if (value > maxAngle && value < 200)
		Servo::write(maxAngle);
	else							// minAngle > value > maxAngle || value > 200
		Servo::write(value);
}
