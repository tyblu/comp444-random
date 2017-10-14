/*
 * TybluServo.cpp
 *
 *  Created on: Oct 14, 2017
 *      Author: tyblu
 */

#include "TybluServo.h"

TybluServo::TybluServo() : Servo()
{
	this->minAngle = 0;
	this->maxAngle = 180;
	this->sensorPin = -1;
}

TybluServo::TybluServo(int min, int max, int sensorPin) : Servo()
{
	this->minAngle = min;
	this->maxAngle = max;
	this->sensorPin = sensorPin;
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
	return -1;
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
