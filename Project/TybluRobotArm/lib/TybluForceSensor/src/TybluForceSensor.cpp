/*
 * TybluForceSensor.cpp
 *
 *  Created on: Oct 13, 2017
 *      Author: tyblu
 */

#include "TybluForceSensor.h"

/*
 *
 */
TybluForceSensor::TybluForceSensor(unsigned int powerPin, unsigned int sensorPin,
		double slope, double offset)
{
	this->slope = slope;
	this->offset = offset;
}

void TybluForceSensor::setSensorSlope(double slope)
{
	this->slope = slope;
}

void TybluForceSensor::setSensorOffset(double offset)
{
	this->offset = offset;
}

int TybluForceSensor::getValue(float varianceLimit)
{
	return 0;
}
