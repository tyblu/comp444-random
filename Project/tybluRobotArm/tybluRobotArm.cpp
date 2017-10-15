#include <Arduino.h>
#include "TybluServo.h"

TybluServo boomArmServo(75, 110, A0, 1.1131363501, -147.1303003827, 50);
int angle = ( boomArmServo.getMaxAngle() + boomArmServo.getMinAngle() ) / 2;
long timestamp = millis();
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
		Serial.println(); delay(5);

		Serial.print("Angle = "); delay(5);
		Serial.print( boomArmServo.read() ); delay(5);
		Serial.print(", Analog Angle = "); delay(5);
		long sampleTime = millis();
		int analogAngle = boomArmServo.getAnalogAngle();
		sampleTime = millis() - sampleTime;
		Serial.print( analogAngle );
		Serial.print(", Time = ");
		Serial.print( sampleTime ); delay(5);
		Serial.print(" ms"); delay(5);
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
