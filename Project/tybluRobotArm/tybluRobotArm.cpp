#include <Arduino.h>
#include "QuickStats.h"

void quickStatsDemo();

void setup()
{
	Serial.begin(9600);
	Serial.println("Compiled " __DATE__ " at " __TIME__);

	Serial.println();
	Serial.println();

	quickStatsDemo();
}

void loop() {}

void quickStatsDemo()
{
	Serial.println("QuickStats Demo");

	float array[] = {1, 1, 2, 3, 3, 3, 3, 3, 4, 5, 6, 7, 8, 9, 100};
	Serial.print("array[] = {");
	for (int i=0; i<15; i++)
	{
		Serial.print(array[i]);
		Serial.print(", ");
	}
	Serial.println("};");

	QuickStats qs;
	Serial.print("Average: ");
	Serial.println( qs.average(array, 15) );
	Serial.print("Geometric Average: ");
	Serial.println( qs.g_average(array, 15) );
	Serial.print("Mode: ");
	Serial.println( qs.mode(array, 15, 0.1) );
}
