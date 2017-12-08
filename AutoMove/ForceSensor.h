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

class ForceSensor
{
public:
	ForceSensor(uint8_t adcPin, uint8_t powerPin, uint16_t minReading, uint16_t maxReading);
	uint8_t read();	// turns sensor on, returns measurement (0-255), returns pin orig. power state
	int raw();	// single AnalogRead value
	void on();	// turns sensor power on
	void off();	// turns sensor power off
	bool isActive();
	//void combineWith(ForceSensor& anotherForceSensor);
	//void remove(ForceSensor& anotherForceSensor);

	static void ForceSensor::report(ForceSensor & sLeft, ForceSensor & sRight, bool lineBreak = false);

private:
	uint8_t adcPin, powerPin;
	uint16_t minReading, maxReading;
	//ForceSensor * combinedList[MAX_COMBINED_SENSORS];
	//uint8_t combinedSensorsCount;

	uint8_t filteredMeasure();
};

#endif

