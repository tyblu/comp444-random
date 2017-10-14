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
	if (timestamp + 1000 > millis() )
	{
		long sampleTime = millis();
		float array[100];
		for (int i=0; i<100; i++)
			array[i] = analogRead(A0);
		sampleTime = millis() - sampleTime;

		Serial.println();

		Serial.print("Angle = ");
		Serial.print( boomArmServo.read() );
		Serial.print(", Sample Time = ");
		Serial.print( sampleTime );
		Serial.println(" ms");

		Serial.print("Average = ");
		sampleTime = millis();
		float avg = qs.average(array, 100);
		sampleTime = millis() - sampleTime;
		Serial.print( avg );
		Serial.print(", Compute Time = ");
		Serial.print( sampleTime );
		Serial.println(" ms");

		Serial.print("  Mode =");
		sampleTime = millis();
		float mode = qs.mode(array, 100, 1);
		sampleTime = millis() - sampleTime;
		Serial.print( mode );
		Serial.print(", Compute Time = ");
		Serial.print( sampleTime );
		Serial.println(" ms");

		Serial.print("Std. Dev.=");
		sampleTime = millis();
		float stdev = qs.stdev(array, 100);
		sampleTime = millis() - sampleTime;
		Serial.print( stdev );
		Serial.print(", Compute Time = ");
		Serial.print( sampleTime );
		Serial.println(" ms");
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
