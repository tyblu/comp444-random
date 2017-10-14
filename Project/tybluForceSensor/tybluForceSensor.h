/*
 * TybluForceSensor.h
 *
 *  Created on: Oct 13, 2017
 *      Author: tyblu
 */

#include <Arduino.h>

#ifndef TYBLUFORCESENSOR_H_
#define TYBLUFORCESENSOR_H_

class TybluForceSensor
{
public:
	TybluForceSensor();

private:
	float slope, offset;
	unsigned int
};

#endif /* TYBLUFORCESENSOR_H_ */
