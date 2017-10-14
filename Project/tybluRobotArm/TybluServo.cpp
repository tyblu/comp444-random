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

