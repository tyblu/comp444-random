/*
 * ForceSensor.h
 *
 *  Created on: Nov 1, 2017
 *      Author: Tyler Lucas
 */

#include "ForceSensor.h"

ForceSensor::ForceSensor(uint8_t adcPin, uint8_t powerPin, uint8_t minReading)
	: adcPin(adcPin)
	, powerPin(powerPin)
	, minReading(minReading)
	//, combinedSensorsCount(0)
{
	pinMode(powerPin, OUTPUT);
	pinMode(adcPin, INPUT);
}

uint8_t ForceSensor::read() 
{
	bool wasActive = isActive();
	on();
	uint8_t result = filteredMeasure();
	if (!wasActive)
		off();
	return result;
}
//uint8_t ForceSensor::read()
//{
//	uint16_t sum = 0;
//	for (int i = 0; i < combinedSensorsCount; i++)
//		sum += combinedList[i]->read();
//	return filteredRead() + (uint8_t)((sum + combinedSensorsCount / 2) / combinedSensorsCount);
//}

void ForceSensor::on()
{
	digitalWrite(powerPin, HIGH);
	delayMicroseconds(2);
}

void ForceSensor::off()
{
	digitalWrite(powerPin, LOW);
	delayMicroseconds(2);
}

bool ForceSensor::isActive()
{
	return (digitalRead(powerPin) == HIGH);
}

// returns 0-255
uint8_t ForceSensor::filteredMeasure()
{
	int dat;
	uint16_t count = 0;
	uint16_t sum = 0;
	uint32_t timeout = millis() + FILTERED_READ_TIMEOUT_MS;
	do {
		dat = analogRead(adcPin);
		if (dat > minReading)
		{
			sum += dat;
			count++;
		}
	} while (count < FILTERED_READ_POINTS && millis() < timeout);

	// convert from 10-bit analogRead to 8-bit value (1023 to 255)
	count *= 4;	// 1023/255 = 4.01, or about 4

	return (uint8_t)((sum + count / 2) / count);	// rounds to nearest int
}

//void ForceSensor::combineWith(ForceSensor& anotherForceSensor)
//{
//	//
//}

//void ForceSensor::remove(ForceSensor& anotherForceSensor)
//{
//	//
//}
