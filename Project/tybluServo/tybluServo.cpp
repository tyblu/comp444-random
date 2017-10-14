/*
 * tybluServo.h - Library for controlling servos on my robotic arm.
 *
 *  Created on: Oct 13, 2017
 *      Author: Tyler Lucas
 */

#include "tybluServo.h"

// Public
tybluServo::tybluServo() : Servo() {}

tybluServo::tybluServo(int minAngle, int maxAngle, tybluForceSensor fs) : Servo()
{
	this->minAngle = minAngle;
	this->maxAngle = maxAngle;
	this->fSensor = fs;
}

void setMinAngle(int minAngle)
{
	// ...
}

void setMaxAngle(int maxAngle)
{
	// ...
}

int getMinAngle()
{
	return 0;
}

int getMaxAngle()
{
	return 0;
}

int getAnalogAngle()
{
	return 0;
}
