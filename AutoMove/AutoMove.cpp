#include <Arduino.h>
#include "inttypes.h"
#include "C:\Users\tyblu\Documents\repos\comp444-random\AutoMove\RobotArmMember.h"
#include "RobotArmState.h"
#include "SonarSensor.h"
#include "SdFat.h"
#include "limits.h"
#include "ForceSensor.h"
#include "AutoMovePinDefinitions.h"
#include "TopoScan.h"
#include "AutoMoveSD.h"

#define AutoMove_DEBUG_MODE
#ifdef AutoMove_DEBUG_MODE
#	define PRE Serial.print(F("AutoMove : "))
#	define POST delay(2) // note missing ';'
#	define DEBUG1(x) PRE; Serial.println(x); POST
#	define DEBUG2(x,y) PRE; Serial.print(x); Serial.println(y); POST
#	define DEBUG3(f,xT,xF) PRE; if(f) { Serial.println(xT); } else { Serial.println(xF); } POST
#else
#	define DEBUG1(x)
#	define DEBUG2(x,y)
#	define DEBUG3(f,xT,xF)
#endif

// Servo stuff.
// TowerPro 946R, sensor needs verification
RobotArmMember memberBoom1(RobotArmMember::ServoName::Boom1, 
	135, -60, BOOM1_PWM_PIN, 100, 150, 110, BOOM1_ADC_PIN, 0.557, -49.64);
// Power HD 1501MG
RobotArmMember memberBoom2(RobotArmMember::ServoName::Boom2, 
	142, -90, BOOM2_PWM_PIN, 70, 115, 80, BOOM2_ADC_PIN, 1.113, -147.1);
// SG90 (micro), not measured -- will probably swap for TowerPro 946R
RobotArmMember memberTurret(RobotArmMember::ServoName::Turret, 
	0, 0, TURRET_PWM_PIN, 30, 150, 90, TURRET_ADC_PIN, 1.000, -1.000);
// TowerPro 946R, angles need verification
RobotArmMember memberClaw(RobotArmMember::ServoName::Claw, 
	0, 0, CLAW_PWM_PIN, 60, 130, 100, CLAW_ADC_PIN, 0.557, -61.28);

RobotArmState state(
	RobotArmState::EndEffectorState::P00Deg,
	SERVO_POWER_CONTROL_PIN, SERVO_POWER_FEEDBACK_PIN,
	memberBoom1, memberBoom2, memberTurret, memberClaw);

// Sonar and SPI SD card stuff.
SonarSensor sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN);
SdFatEX sd;
SdFile file;
TopoScan topoScan(state, sonar);

// Force sensor stuff.
ForceSensor sensorL(FORCE_SENSOR_ANALOG_A_PIN, FORCE_SENSORS_POWER_PIN, 10);
ForceSensor sensorR(FORCE_SENSOR_ANALOG_B_PIN, FORCE_SENSORS_POWER_PIN, 10);

void setup()
{
	Serial.begin(9600);
	Serial.println(__FILE__ " compiled " __DATE__ " at " __TIME__);
	Serial.println();

	// Servo stuff.

	/* The following should probably be moved to a RobotArmState function. */
	while (!state.list.isFinished())
	{
		RobotArmMember servo = state.getServo(state.list.current());
		servo.write(servo.getSafeAngle());
		servo.attach();

		DEBUG3(servo.attached(), F("Servo attached."), F("Servo failed to attach."));

		state.list.next();
	}

	// power on servos
	/* The following should probably be moved to a RobotArmState function. */
	digitalWrite(SERVO_POWER_CONTROL_PIN, HIGH);

	/* The following should probably be moved to a RobotArmState function. */
	state.list.restart();
	while (!state.list.isFinished())
	{
		RobotArmMember servo = state.getServo(state.list.current());
		servo.smooth(servo.getMinAngle());
		servo.smooth(servo.getMaxAngle());
		servo.smooth(servo.getSafeAngle());

		DEBUG1(F("Finished wiggling a servo."));

		state.list.next();
	}

	// SD Card stuff
	/* The following should probably be moved to a class. AutoMoveSD? */
	DEBUG1(F("Starting SPI SD card stuff."));
	/* Initialize at the highest speed supported by the board that is
	// not over 50 MHz. Try a lower speed if SPI errors occur. */
	DEBUG3(!sd.begin(SD_CS_PIN, SD_SCK_MHZ(50)), F("begin failed."), F("begin success."));
#ifndef AutoMove_DEBUG_MODE
	if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(50))
		sd.initErrorHalt();
#endif

	char folderDir[] = "/TopoMaps";
	if (!sd.exists(folderDir))
	{
		DEBUG3(sd.mkdir(folderDir), F("Create \"TopoMaps\" succeeded."), F("Create \"TopoMaps\" failed."));
#ifndef AutoMove_DEBUG_MODE
		sd.mkdir(folderName);
#endif
	}

	char filename[13];
	getUniqueFileNameIndex(filename, sd, folderDir, "sonar", "txt");
	DEBUG2(F("Unique file name: "), filename);

#ifdef AutoMove_DEBUG_MODE
	DEBUG3(!sd.chdir(folderDir), F("chdir to \"TopoMaps\" failed."), F("chdir success."));
	DEBUG3(!file.open(filename, O_CREAT | O_WRITE), F("Open failed."), F("Open success."));
#else
	sd.chdir(folderDir);
	file.open(filename, O_CREAT);
#endif
	
	//logSonarDataHeader(file);
	//logSonarDataHeaderEverything(file);
	topoScan.logSonarDataHeaderEverything(file);

	// Force sensor stuff.
	DEBUG2(F("LHS sensor reading: "), sensorL.read());
	DEBUG2(F("RHS sensor reading: "), sensorR.read());
}

void loop()
{
	// Go to middle of paper and calibrate height.
	/* The following should probably be moved to a class function.
	 * RobotArmState or TopoScan? Proabably TopoScan. */
	memberBoom2.smooth(100);
	memberBoom1.smooth(115);
	delay(500);
	uint32_t heightZero = sonar.getMeasurement();

	/* The following should probably be moved to TopoScan. */
	for (uint8_t rad = memberBoom1.getMinAngle(); rad < memberBoom1.getMaxAngle(); rad += 5)
	{
		memberBoom1.smooth(rad);
		memberTurret.smooth(90);

		// zero height with boom2
		int32_t heightDiff = sonar.getMeasurement() - heightZero;
		uint32_t timeout = millis() + 500;
		while (heightDiff > 25 && millis() < timeout)
		{
			memberBoom2.smooth((heightDiff > 0) ? -1 : 1);
			heightDiff = sonar.getMeasurement() - heightZero;
		}

		//int32_t timeDiff;
		//do {
		//	timeDiff = micros() - logTime;
		//} while (timeDiff < 0); // wait for log time

		for (uint8_t theta = memberTurret.getMinAngle(); theta < memberTurret.getMaxAngle(); theta += 5)
		{
			memberTurret.smooth(theta);
			//logSonarDataEverything(sonar, file, state);
			topoScan.logSonarDataEverything(file);
			file.sync();
		}

		for (uint8_t theta = memberTurret.getMaxAngle(); theta > memberTurret.getMinAngle(); theta -= 5)
		{
			memberTurret.smooth(theta);
			//logSonarDataEverything(sonar, file, state);
			topoScan.logSonarDataEverything(file);
			file.sync();
		}
	}
}