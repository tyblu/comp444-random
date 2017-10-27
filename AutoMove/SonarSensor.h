/*
* SonarSensor.h
*
*  Created on: Oct 26, 2017
*      Author: Tyler Lucas <tyblu@live.com>
*/

#ifndef SonarSensor_h
#define SonarSensor_h

#include <inttypes.h>

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class SonarSensor
{
public:
	SonarSensor(int triggerPin, int echoPin);

	long getMeasurement();

private:
	int triggerPin, echoPin;
};

#endif

