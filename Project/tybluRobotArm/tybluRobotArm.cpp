#include <Arduino.h>
#include "TybluServo.h"

//TybluServo(minAngle, maxAngle, sensorPin, sensorSlope, sensorOffset, pt_count);
TybluServo boomArmServo(75, 110, A0, 1.13539908, -145.580693, 50);
int angle = ( boomArmServo.getMaxAngle() + boomArmServo.getMinAngle() ) / 2;
long timestamp = millis();
int angleAdjustment = 1;

void setup()
{
	Serial.begin(9600);
	Serial.println("Compiled " __DATE__ " at " __TIME__);
	Serial.println();

	boomArmServo.write(angle);
	boomArmServo.attach(11);

	Serial.println("INPUT ANGLE, MEASURED ANGLE, MEASUREMENT TIME");

	delay(1000);
}

void loop()
{
	if (timestamp + 1000 > millis() )
	{
		Serial.print( boomArmServo.read() );
		Serial.write(',');
		long sampleTime = millis();
		int analogAngle = boomArmServo.getAnalogAngle();
		sampleTime = millis() - sampleTime;
		Serial.print( analogAngle );
		Serial.write(',');
		Serial.print( sampleTime );
		Serial.println();
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
