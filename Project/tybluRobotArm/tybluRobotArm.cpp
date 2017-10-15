#include <Arduino.h>
#include "QuickStats.h"
#include "TybluServo.h"

TybluServo boomArmServo(75, 110, A0);
int angle = (115+70)/2;
long timestamp = millis();
QuickStats qs;
int angleAdjustment = 3;

void setup()
{
	Serial.begin(9600);
	Serial.println("Compiled " __DATE__ " at " __TIME__);
	Serial.println();

	boomArmServo.write(angle);
	boomArmServo.attach(11);

	delay(1000);
}

void loop()
{
	const int POINTS = 75;
	if (timestamp + 1000 > millis() )
	{
		Serial.println();

		Serial.print("             Angle = "); delay(5);
		Serial.print( boomArmServo.read() ); delay(5);
		Serial.print(",    Sample Time = "); delay(5);
		float array[POINTS];
		long sampleTime = millis();
		for (int i=0; i<POINTS; i++)
			array[i] = analogRead(A0);
		sampleTime = millis() - sampleTime;
		Serial.print( sampleTime ); delay(5);
		Serial.print(" ms,         Sorting Time = "); delay(5);
		float arraySorted[POINTS];
		for (int i=0; i<POINTS; i++)
			arraySorted[i] = array[i];
		sampleTime = millis();
		qs.bubbleSort(arraySorted, POINTS);
		sampleTime = millis() - sampleTime;
		Serial.print( sampleTime ); delay(5);
		Serial.print(" ms"); delay(5);
		Serial.println(); delay(5);

		Serial.print("           Average = "); delay(5);
		sampleTime = millis();
		float avg = qs.average(array, POINTS);
		sampleTime = millis() - sampleTime;
		Serial.print( avg ); delay(5);
		Serial.print(", Compute Time = "); delay(5);
		Serial.print( sampleTime ); delay(5);
		Serial.print(" ms, Compute Time (sorted) = "); delay(5);
		sampleTime = millis();
		avg = qs.average(arraySorted, POINTS);
		sampleTime = millis() - sampleTime;
		Serial.print( sampleTime ); delay(5);
		Serial.print(" ms"); delay(5);
		Serial.println(); delay(5);

		Serial.print("              Mode = "); delay(5);
		sampleTime = millis();
		float mode = qs.mode(array, POINTS, 1);
		sampleTime = millis() - sampleTime;
		Serial.print( mode ); delay(5);
		Serial.print(", Compute Time = "); delay(5);
		Serial.print( sampleTime ); delay(5);
		Serial.print(" ms, Compute Time \(sorted\) = "); delay(5);
		sampleTime = millis();
		avg = qs.mode(arraySorted, POINTS, 1);
		sampleTime = millis() - sampleTime;
		Serial.print( sampleTime ); delay(5);
		Serial.print(" ms"); delay(5);
		Serial.println(); delay(5);

		Serial.print("Standard Deviation = "); delay(5);
		sampleTime = millis();
		float stdev = qs.stdev(array, POINTS);
		sampleTime = millis() - sampleTime;
		Serial.print( stdev ); delay(5);
		Serial.print(", Compute Time = "); delay(5);
		Serial.print( sampleTime ); delay(5);
		Serial.print(" ms, Compute Time \(sorted\) = "); delay(5);
		sampleTime = millis();
		avg = qs.stdev(arraySorted, POINTS);
		sampleTime = millis() - sampleTime;
		Serial.print( sampleTime ); delay(5);
		Serial.print(" ms"); delay(5);
		Serial.println(); delay(5);
	}
	else
	{
		if (angle >= boomArmServo.getMaxAngle() || angle <= boomArmServo.getMinAngle() )
			angleAdjustment *= -1;
		angle += angleAdjustment;
		boomArmServo.write(angle);
		delay(1000);
		timestamp = millis();
	}
}
