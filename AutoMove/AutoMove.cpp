#include <Arduino.h>
#include "C:\Users\tyblu\Documents\repos\comp444-random\TybluServo\TybluServo.h"

#define AutoMove_DEBUG_MODE
#ifdef AutoMove_DEBUG_MODE
#	define DEBUG1(x) Serial.print("AutoMove : "); Serial.println(x); delay(2)	// note missing ';'
#	define DEBUG2(x,y) Serial.print("AutoMove : "); Serial.print(x); Serial.println(y); delay(2)	// note missing ';'
#else
#	define DEBUG1(x)
#	define DEBUG2(x,y)
#endif

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
 */
TybluServo boom1Servo(BOOM1_PWM_PIN, 100, 150,  110, A1, 0.557, -49.64);	// TowerPro 946R, sensor needs verification
TybluServo boom2Servo(BOOM2_PWM_PIN, 70, 115,  80, A0, 1.113, -147.1);	// Power HD 1501MG
//TybluServo turretServo (TURRET_PWM_PIN, 30, 150,  90, A3, 1.000, -1.000);	// not measured
//TybluServo clawServo (CLAW_PWM_PIN, 80, 130, 100, A2, 0.557, -61.38);	// TowerPro 946R, angles need verification
#define NUM_ACTIVE_SERVOS 2
TybluServo * servos[NUM_ACTIVE_SERVOS] = { &boom1Servo , &boom2Servo };

void setup()
{
	delay(2000);

	Serial.begin(9600);
	Serial.println(__FILE__ " compiled " __DATE__ " at " __TIME__);
	Serial.println();

	for (int i=0; i<NUM_ACTIVE_SERVOS; i++)
	{
		unsigned int servoAttachAttempts = 0;
		do {
			servos[i]->attach();
			if (servoAttachAttempts++ > 3)
			{
				DEBUG1("Could not attach a servo.");
				break;
			}
			delay(10);
		} while (!servos[i]->attached() );

		DEBUG1("Calibrating Sensor...");

		servos[i]->calibrateSensor();

		DEBUG1(" Finished calibrating a servo. ");

		servos[i]->smooth( servos[i]->getSafeAngle() );

		Serial.println();
		servos[i]->printSensorLine();
		Serial.println(); delay(5);

		DEBUG1("Next servo (if any)...");
		Serial.println();
	}
	Serial.println();

	delay(2000);
}

void loop()
{
	DEBUG1("Made it!");
	delay(5000);
}
