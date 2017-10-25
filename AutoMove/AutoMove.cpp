#include <Arduino.h>

#include "C:\Users\tyblu\Documents\repos\comp444-random\TybluServo\TybluServo.h"

#define BOOM1_PWM_PIN 6
#define BOOM2_PWM_PIN 9
#define CLAW_PWM_PIN 5
#define TURRET_PWM_PIN 3
/*
 * TybluServo constructor arguments: (int, int, int, int, int, float, float, int)
 * int pwmPin								Pin used to control servo.
 * int minAngle, int maxAngle 				Allowed range of movement.
 * int safeAngle							Nominal position for servo.
 * int sensorPin 							Angle sensor pin (A0, A1, ...).
 * float sensorSlope, float sensorOffset	Initial angle sensor linear coefficients.
 * int pt_count								Number of float measurements to use when
 * 											calibrating angle sensor. NOTE: Should
 *		hard-code this and use other memory (PROGMEM, etc.) to get around this
 *		memory constraint.
 */
TybluServo boom1Servo (BOOM1_PWM_PIN, 100, 150,  90, A1, 0.557, -49.64);	// TowerPro 946R, sensor needs verification
TybluServo boom2Servo (BOOM2_PWM_PIN, 70, 115,  80, A0, 1.113, -147.1);	// Power HD 1501MG
//TybluServo turretServo (TURRET_PWM_PIN, 30, 150,  90, A3, 1.000, -1.000);	// not measured
//TybluServo clawServo (CLAW_PWM_PIN, 80, 130, 100, A2, 0.557, -61.38);	// TowerPro 946R, angles need verification
#define NUM_ACTIVE_SERVOS 2
TybluServo * servos[NUM_ACTIVE_SERVOS] = { &boom2Servo , &boom1Servo };

void dots(int n, int t);
void ellipsis();

void setup()
{
	delay(2000);

	Serial.begin(9600);
	Serial.println(__FILE__ " compiled " __DATE__ " at " __TIME__);
	Serial.println();

	Serial.print("Calibrating Sensors"); ellipsis(); Serial.println();
	for (int i=0; i<NUM_ACTIVE_SERVOS; i++)
	{
		unsigned int servoAttachAttempts = 0;
		do {
			servos[i]->attach();
			if (servoAttachAttempts > 3)
			{
				Serial.println("Could not attach a servo.");
				break;
			}
		} while (!servos[i]->attached() );

		servos[i]->calibrateSensor();

		Serial.println(" Finished calibrating a servo. ");
		servos[i]->smooth( servos[i]->getSafeAngle() );

		Serial.println();
		Serial.print(i);
		Serial.print(": y = ");
		Serial.print(servos[i]->getSensorSlope());
		Serial.print(" * x");
		if (servos[i]->getSensorOffset() < 0)
			Serial.print(" - ");
		else
			Serial.print(" + ");
		Serial.print( abs( servos[i]->getSensorOffset() ) );
	}
	Serial.println();

	delay(2000);
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

