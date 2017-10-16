#include <Arduino.h>
#include "TybluServo.h"

TybluServo boomArmServo(70, 115, A0, 1, 0, 50);
int angle = ( boomArmServo.getMaxAngle() + boomArmServo.getMinAngle() ) / 2;
long timestamp = millis();
int angleAdjustment = 4;

void dots(int n, int t);
void ellipsis();

void setup()
{
	delay(2000);

	Serial.begin(9600);
	Serial.println(__FILE__ " compiled " __DATE__ " at " __TIME__);
	Serial.println();

	boomArmServo.attach(11);

	Serial.print("Calibrating Sensor"); ellipsis();
	const int angleA = boomArmServo.getMinAngle() + 1;
	const int angleB = boomArmServo.getMaxAngle() - 1;
	boomArmServo.calibrateSensor(angleA, angleB);
	Serial.println();
	Serial.print("y = ");
	Serial.print(boomArmServo.getSensorSlope());
	Serial.print(" * x ");
	float offset = boomArmServo.getSensorOffset();
	if (offset < 0)
		Serial.print("- ");
	else
		Serial.print("+ ");
	Serial.print(abs(offset));
	Serial.println();

	delay(2000);

	boomArmServo.detach();
}

void loop()
{
	Serial.println("Made it!");
	delay(5000);
//	if (timestamp + 1000 > millis() )
//	{
//		Serial.print( boomArmServo.read() );
//		Serial.write(',');
//		long sampleTime = millis();
//		int analogAngle = boomArmServo.getAnalogAngle();
//		sampleTime = millis() - sampleTime;
//		Serial.print( analogAngle );
//		Serial.write(',');
//		Serial.print( sampleTime );
//		Serial.println();
//	}
//	else
//	{
//		if (angle >= boomArmServo.getMaxAngle() || angle <= boomArmServo.getMinAngle() )
//			angleAdjustment *= -1;
//		angle += angleAdjustment;
//		boomArmServo.write(angle);
//		delay(1000);
//		timestamp = millis();
//	}
}

void dots(int num, int delayTime)
{
	for (int i=0; i<num; i++)
	{
		Serial.write('.');
		delay(delayTime);
	}
}

void ellipsis() { dots(3, 100); }
