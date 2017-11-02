/*
 * ForceSensor.h
 *
 *  Created on: Nov 1, 2017
 *      Author: Tyler Lucas
 */

#ifndef ForceSensor_h
#define ForceSensor_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "inttypes.h"

#define MAX_COMBINED_SENSORS 5
#define FILTERED_READ_POINTS 50
#define FILTERED_READ_TIMEOUT_MS 100

class ForceSensor
{
public:
	ForceSensor(uint8_t adcPin, uint8_t powerPin, uint8_t minReading);
	uint8_t read();	// turns sensor on, returns measurement (0-255), returns pin orig. power state
	void on();	// turns sensor power on
	void off();	// turns sensor power off
	bool isActive();
	//void combineWith(ForceSensor& anotherForceSensor);
	//void remove(ForceSensor& anotherForceSensor);

private:
	uint8_t adcPin, powerPin;
	uint8_t minReading;
	//ForceSensor * combinedList[MAX_COMBINED_SENSORS];
	//uint8_t combinedSensorsCount;

	uint8_t filteredMeasure();
};

#endif

