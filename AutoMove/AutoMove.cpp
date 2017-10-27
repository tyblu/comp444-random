#include <Arduino.h>
#include "TybluServo.h"
#include "SonarSensor.h"
#include "SdFat.h"

#define AutoMove_DEBUG_MODE
#ifdef AutoMove_DEBUG_MODE
#	define DEBUG1(x) Serial.print("AutoMove : "); Serial.println(x); delay(2)	// note missing ';'
#	define DEBUG2(x,y) Serial.print("AutoMove : "); Serial.print(x); Serial.println(y); delay(2)	// note missing ';'
#else
#	define DEBUG1(x)
#	define DEBUG2(x,y)
#endif

// Servo stuff.
//#define SERVO_ON
#ifdef SERVO_ON
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
TybluServo turretServo (TURRET_PWM_PIN, 30, 150,  90, A3, 1.000, -1.000);	// not measured
TybluServo clawServo (CLAW_PWM_PIN, 60, 130, 100, A2, 0.557, -61.38);	// TowerPro 946R, angles need verification
#define NUM_ACTIVE_SERVOS 4
TybluServo * servos[NUM_ACTIVE_SERVOS] = 
		{ &boom2Servo , &boom1Servo, &turretServo, &clawServo };
#endif // SERVO_ON

// Sonar stuff.
#define SONAR_ON
#ifdef SONAR_ON
#define SONAR_TRIGGER_PIN 4
#define SONAR_ECHO_PIN 7
SonarSensor sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN);
#endif	// SONAR_ON

// SPI SD card stuff.
#define SD_ON
#ifdef SD_ON
#define SD_CS_PIN 10
#define SD_SCK_PIN 13	// ICSP #3
#define SD_MOSI_PIN 11	// ICSP #4
#define SD_MISO_PIN 12	// ICSP #1
#endif // SD_ON

void setup()
{
	Serial.begin(9600);
	Serial.println(__FILE__ " compiled " __DATE__ " at " __TIME__);
	Serial.println();

	// Servo stuff.
#ifdef SERVO_ON
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

		servos[i]->smooth(servos[i]->getSafeAngle());
		servos[i]->smooth(servos[i]->getMaxAngle());
		servos[i]->smooth(servos[i]->getMinAngle());
		servos[i]->smooth(servos[i]->getSafeAngle());

		DEBUG1("Next servo (if any)...");
	}
	Serial.println();
#endif // SERVO_ON

	// SD Card stuff
#ifdef SONAR_ON

#endif // SONAR_ON

}

void loop()
{
	// SD Card stuff
#ifdef SONAR_ON

#endif // SONAR_ON

}
