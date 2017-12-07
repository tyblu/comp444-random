/*
 * ForceSensor.h
 *
 *  Created on: Nov 1, 2017
 *      Author: Tyler Lucas
 */

#include "ForceSensor.h"

#define MAX_COMBINED_SENSORS 5
#define FILTERED_READ_POINTS 50
#define FILTERED_READ_TIMEOUT_MS 100

//#define ForceSensor_DEBUG_MODE
#ifdef ForceSensor_DEBUG_MODE
#	define PRE Serial.print(F("ForceSensor: "))
#	define POST delay(5)
#	define BREAK Serial.println()
#	define DEBUG(x)	(x); POST
#	define DEBUGLN(x)	PRE; (x); BREAK; POST
#else
#	define PRE 
#	define POST
#	define BREAK
#	define DEBUG(x)
#	define DEBUGLN(x) 
#endif // ForceSensor_DEBUG_MODE

ForceSensor::ForceSensor(uint8_t adcPin, uint8_t powerPin, uint16_t minReading, uint16_t maxReading)
	: adcPin(adcPin)
	, powerPin(powerPin)
	, minReading(minReading)
	, maxReading(maxReading)
	//, combinedSensorsCount(0)
{
	pinMode(powerPin, OUTPUT);
	pinMode(adcPin, INPUT);
}

uint8_t ForceSensor::read() 
{
	bool wasActive = isActive();

	if (!wasActive)
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

int ForceSensor::raw()
{
	return analogRead(adcPin);
}

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

	BREAK; PRE;
	DEBUG(Serial.print(F("millis() before = ")));
	DEBUG(Serial.print(millis()));
	POST; BREAK;

	do {
		dat = analogRead(adcPin);

		//BREAK; PRE;
		//DEBUG(Serial.print(F("filteredMeasure() do-while: dat=")));
		//DEBUG(Serial.print(dat));
		//DEBUG(Serial.print(F(", (dat > minReading && dat < maxReading)=(")));
		//DEBUG(Serial.print((dat > minReading) ? (F("TRUE && ")) : (F("FALSE && "))));
		//DEBUG(Serial.print((dat < maxReading) ? (F("TRUE)=")) : (F("FALSE)="))));
		//DEBUG(Serial.print((dat > minReading && dat < maxReading) ? (F("TRUE, sum=")) : F("FALSE, sum=")));

		if (dat > minReading && dat < maxReading)
		{
			sum += dat;
			count++;
		}

		//DEBUG(Serial.print(sum));
		//DEBUG(Serial.print(F(", count=")));
		//DEBUG(Serial.print(count));
		//POST; BREAK; PRE;
		//DEBUG(Serial.print(F("minReading: ")));
		//DEBUG(Serial.print(minReading));
		//DEBUG(Serial.print(F(", maxReading: ")));
		//DEBUG(Serial.print(maxReading));
		//POST; BREAK;
		//
		//PRE;
		//DEBUG(Serial.print(F("(count < FILTERED_READ_POINTS && millis() < timeout)=(")));
		//DEBUG(Serial.print(count));
		//DEBUG(Serial.print(F(" < ")));
		//DEBUG(Serial.print(FILTERED_READ_POINTS));
		//DEBUG(Serial.print(F(" && ")));
		//DEBUG(Serial.print(millis()));
		//DEBUG(Serial.print(F(" < ")));
		//DEBUG(Serial.print(timeout));
		//DEBUG(Serial.print(F(")=(")));
		//DEBUG(Serial.print((count < FILTERED_READ_POINTS) ? (F("TRUE && ")) : (F("FALSE && "))));
		//DEBUG(Serial.print((millis() < timeout) ? (F("TRUE)=")) : (F("FALSE)="))));
		//DEBUG(Serial.print((count < FILTERED_READ_POINTS && millis() < timeout) ? (F("TRUE")) : (F("FALSE"))));
		//POST; BREAK;

	} while (count < FILTERED_READ_POINTS && millis() < timeout);

	BREAK; PRE;
	DEBUG(Serial.print(F("millis()  after = ")));
	DEBUG(Serial.print(millis()));
	POST; BREAK;

	BREAK; PRE;
	DEBUG(Serial.print(F("filteredMeasure(): count=")));
	DEBUG(Serial.print(count));
	DEBUG(Serial.print(F(", sum=")));
	DEBUG(Serial.print(sum));
	DEBUG(Serial.print(F(", timeout? ")));
	DEBUG(Serial.print((millis() < timeout) ? (F("no")) : (F("yes"))));
	BREAK; POST;

	// convert from 10-bit analogRead to 8-bit value (1023 to 254)
	count *= 4;	// 1023/254 = 4.03, or about 4
	
	DEBUGLN(Serial.print(F("(uint8_t)((sum+count*4/2)/count*4)=")));
	PRE;
	DEBUG(Serial.print(F("(uint8_t)((sum+")));
	DEBUG(Serial.print(count));
	DEBUG(Serial.print(F("/2)/")));
	DEBUG(Serial.print(count));
	DEBUG(Serial.print(F(")=")));
	BREAK; POST; PRE;
	DEBUG(Serial.print(F("(uint8_t)((")));
	DEBUG(Serial.print(sum));
	DEBUG(Serial.print(F("+")));
	DEBUG(Serial.print(count / 2));
	DEBUG(Serial.print(F(")/")));
	DEBUG(Serial.print(count));
	DEBUG(Serial.print(F(")=")));
	BREAK; POST; PRE;
	DEBUG(Serial.print(F("(uint8_t)(")));
	DEBUG(Serial.print(sum + count / 2));
	DEBUG(Serial.print(F(")/")));
	DEBUG(Serial.print(count));
	DEBUG(Serial.print(F(")=")));
	BREAK; POST; PRE;
	DEBUG(Serial.print(F("(uint8_t)(")));
	DEBUG(Serial.print((sum + count / 2) / count));
	DEBUG(Serial.print(F(")=")));
	BREAK; POST;
	DEBUGLN(Serial.print((uint8_t)((sum + count / 2) / count)));
	PRE;
	DEBUG(Serial.print(F("returning ")));
	DEBUG(Serial.print((count == 0) ? (0) : ((uint8_t)((sum + count / 2) / count))));
	BREAK; POST;

	if (count == 0)
		return 0;

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

void ForceSensor::report(ForceSensor & sLeft, ForceSensor & sRight, bool lineBreak)
{
	Serial.print(F("[left] "));
	Serial.print(sLeft.read());
	Serial.print(F(" (raw: "));
	Serial.print(sLeft.raw());
	Serial.print(F("), "));
	Serial.print(F("[right] "));
	Serial.print(sRight.read());
	Serial.print(F(" (raw: "));
	Serial.print(sRight.raw());
	Serial.print(F(")"));

	if (lineBreak)
		Serial.println();
}