/*
 * TybluServo.cpp
 *
 *  Created on: Oct 14, 2017
 *      Author: tyblu
 */

#include "TybluServo.h"
#include "QuickStats.h"
#include "TybluLsq.h"

// used in getAnalogAngle and calibrateSensor
#define MODE_EPSILON 1.0
#define MODE_LOWER_LIMIT 10		// 5V *  10/1024 = 49mV
#define MODE_UPPER_LIMIT 512	// 5V * 512/1024 = 2.5V, max in equal voltage divider
#define CALIB_STEP_SIZE 2
#define CALIB_PASSES 1

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

/**
 * Calibrates servo position sensor by fitting line to measured results while moving
 * between angleA and angleB. Movements starts at angleA, travels to angleB, back to
 * angleA, and it may repeat this several more times. The servo always obeys the
 * minAngle and maxAngle limits, so make sure angleA and angleB are within them or
 * the calibration will not work. At the moment, the function just quits if angleA/B
 * are not inside the limits.
 */
void TybluServo::calibrateSensor(int angleA, int angleB)
{
	{	// ensure angleA <= angleB
		int temp = min(angleA, angleB);
		angleB = max(angleA, angleB);
		angleA = temp;
	}

	// invalid sensor pin or calibration test range outside of servo min/max
	if (sensorPin < 0 || angleA < minAngle || angleB > maxAngle)
		return;

/* This should be changed to be a gentle transition. Later. */
	this->write(angleA);
	delay(500);				// wait for servo to reach angleA

	// measure data
	unsigned int arraySize = 2 * CALIB_PASSES * (angleB - angleA) / CALIB_STEP_SIZE + 1;
	float angles[arraySize], measurements[arraySize];
	angles[0] = angleA;
	int direction = 1;
	unsigned int iterator = 0;
	do {
		this->write(angles[iterator]);
		delay(100);
		measurements[iterator] = this->getAnalogAngle();

		Serial.println(); Serial.print(iterator); Serial.print(" : "); Serial.print(angles[iterator]); Serial.print(" | "); Serial.print(measurements[iterator]);

		// skips bad data by not advancing iterator, leading to overwrite
		if (measurements[iterator] > MODE_LOWER_LIMIT
				&& measurements[iterator] < MODE_UPPER_LIMIT)
			iterator++;

		if (iterator == 0)
			continue;

		if (angles[iterator-1] <= angleA)
			direction = 1;
		else if (angles[iterator-1] >= angleB)
			direction = -1;

		angles[iterator] = angles[iterator-1] + direction * CALIB_STEP_SIZE;

//		Serial.print(", ANGLEB="); Serial.print(angles[iterator-1]);
	} while (iterator < arraySize);
	Serial.println();

//	Serial.println();
//	Serial.println("i : angle | meas.");
//	for (int j=0; j<arraySize; j++)
//	{
//		Serial.print(j);
//		Serial.print(" : ");
//		Serial.print(angles[j]);
//		Serial.print(" | ");
//		Serial.print(measurements[j]);
//		Serial.println();
//	}
//	Serial.println();

	// least squares fit
	float a, b;
	TybluLsq lsq;
	// void TybluLsq::llsq( int n, float x[], float y[], float &a, float &b )
	lsq.llsq(arraySize, measurements, angles, a, b);

//	Serial.print(" Y[] = a*X[] + b --> a = ");
//	Serial.print(a);
//	Serial.print(", b = ");
//	Serial.print(b);
//	Serial.println(); delay(5);

	sensorSlope = a;
	sensorOffset = b;
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

float TybluServo::getSensorSlope()
{
	return sensorSlope;
}

float TybluServo::getSensorOffset()
{
	return sensorOffset;
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
