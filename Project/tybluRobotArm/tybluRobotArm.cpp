#include <Arduino.h>
#include "TybluServo.h"

TybluServo boomArmServo(85, 95, A0);

void setup()
{
	Serial.begin(9600);
	boomArmServo.write((70+115)/2);
	boomArmServo.attach(11);
}

void loop()
{
	int angle = (70+115)/2 + (115-70)/3 * sin(millis());
	boomArmServo.write(angle);

	Serial.println();
	Serial.print("Version 0.21 functioning! Input angle: ");
	Serial.print(angle);
	Serial.print("   Angle written: ");
	Serial.print(boomArmServo.read());

	delay(2000);
}
