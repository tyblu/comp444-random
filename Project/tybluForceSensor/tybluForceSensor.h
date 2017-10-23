/*
 * TybluForceSensor.h
 *
 *  Created on: Oct 13, 2017
 *      Author: Tyler Lucas
 */

#include <Arduino.h>

#ifndef TYBLUFORCESENSOR_H_
#define TYBLUFORCESENSOR_H_

class TybluForceSensor
{
public:
	TybluForceSensor(unsigned int powerPin, unsigned int sensorPin,
			double slope = 1, double offset = 0);
	void setSensorSlope(double slope);
	void setSensorOffset(double offset);
	int getValue(float varianceLimit);

private:
	double slope, offset;
};

#endif /* TYBLUFORCESENSOR_H_ */
