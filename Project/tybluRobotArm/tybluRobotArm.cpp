#include <Arduino.h>
#include "TybluServo.h"
#include "QuickStats.h"

TybluServo boomArmServo(75, 110, A0);
float sinCount = 0;

void setup()
{
	Serial.begin(9600);
	boomArmServo.write((70+115)/2);
	boomArmServo.attach(11);

	Serial.println("Compiled " __DATE__ " at " __TIME__);
}

void loop()
{
	sinCount += 0.1;
	int angle = (70+115)/2 + (115-70)/2 * sin(sinCount);
	boomArmServo.write(angle);

	Serial.println();
	Serial.print("Input angle: ");
	Serial.print(angle);
	Serial.print("   Angle written: ");
	Serial.print(boomArmServo.read());
	Serial.print("   sinCount=");
	Serial.print(sinCount);

	delay(500);
}
