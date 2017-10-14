#include <Arduino.h>
#include "QuickStats.h"
#include "TybluServo.h"

TybluServo boomArmServo(75, 110, A0);

void setup()
{
	Serial.begin(9600);
	Serial.println("Compiled " __DATE__ " at " __TIME__);
}

void loop()
{

}
