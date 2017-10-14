#include <Arduino.h>
#include "TybluServo.h"

TybluServo boomArmServo(70, 115, A0);

void setup()
{
	Serial.begin(9600);
	boomArmServo.write((70+115)/2);
	boomArmServo.attach(11);
}

void loop()
{
	Serial.println("Version 0.2 functioning!");
	delay(2000);
	boomArmServo.write((70+115)/2 + (115-70)/2 * sin(millis()));
}
