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

//#define AUTONOMOUS_OPERATION		// comment out for serial (USB) control mode
#ifndef AUTONOMOUS_OPERATION
#	define SERIAL_CONTROL_MODE
#endif

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

#define RAM RobotArmMember
#define RAS RobotArmState

// Servo stuff.
#define CLAW_INCLINE 0
#define CLAW_LENGTH_MIN 40L
#define CLAW_LENGTH_ADJ 30L
#define CLAW_ANGLE_SCALE 1000L
#define CLAW_ANGLE_OFFSET 0L

#define BOOM1_LENGTH 135L
#define BOOM1_ANGLE_SCALE -1000L
#define BOOM1_ANGLE_OFFSET 225L		// 135 + 90

#define BOOM2_LENGTH 142L
#define BOOM2_ANGLE_SCALE 1000L
#define BOOM2_ANGLE_OFFSET -125L

#define TURRET_ANGLE_SCALE 500L
#define TURRET_ANGLE_OFFSET 0L

#define PAIR_BOOM2MINS_COUNT 10

PositionVector presetPositions[] =
{
	{ 125, 120, 135 },		// CenterSonar
	{ 125, 120, 125 },		// Center
	{  90,  70, 135 },		// Rest
	{ 130, 125, 135 },		// MaxHeight
	{   0, -90,   0 },		// MinHeight
	{   0,   0, 135 },		// MaxRadius
	{ 140, -70,   0 }		// MinRadius
};

Pair boom2mins[PAIR_BOOM2MINS_COUNT] = 
{
	{ 90, 83 },
	{ 100, 68 },
	{ 110, 55 },
	{ 120, 50 },
	{ 130, 40 },
	{ 140, 30 },
	{ 150, 20 },
	{ 160, 25 },
	{ 170, 26 },
	{ 180, 26 } 
};

BoomPositionVector posBoom1(BOOM1_LENGTH);
BoomPositionVector posBoom2(BOOM2_LENGTH);
TurretPositionVector posTurret = {};
ClawPositionVector posClaw(CLAW_LENGTH_MIN, CLAW_LENGTH_ADJ, CLAW_INCLINE);

RobotArmMember memberBoom1(BOOM1_PWM_PIN, posBoom1);	// TowerPro 946R
RobotArmMember memberBoom2(BOOM2_PWM_PIN, posBoom2);	// Power HD 1501MG
RobotArmMember memberTurret(TURRET_PWM_PIN, posTurret);	// TowerPro 946R
RobotArmMember memberClaw(CLAW_PWM_PIN, posClaw);		// TowerPro 946R

RobotArmState state(
	SERVO_POWER_CONTROL_PIN, SERVO_POWER_FEEDBACK_PIN,
	memberBoom1, memberBoom2, memberTurret, memberClaw,
	presetPositions, boom2mins
);

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
	memberBoom1.setAngleConstants(BOOM1_ANGLE_SCALE, BOOM1_ANGLE_OFFSET);
	memberBoom2.setAngleConstants(BOOM2_ANGLE_SCALE, BOOM2_ANGLE_OFFSET);
	state.attachSafe();
	state.servoPowerOn();
	state.sweep();
	DEBUG1(F("Finished powering and wiggling servos."));

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
	
	topoScan.logSonarDataHeaderEverything(file);

	// Force sensor stuff.
	DEBUG2(F("LHS sensor reading: "), sensorL.read());
	DEBUG2(F("RHS sensor reading: "), sensorR.read());
}

#ifdef SERIAL_CONTROL_MODE			// arm controlled by USB serial commands
#define PLN(x) Serial.println(F(x)); delay(2)
#define PLN2(x,y) Serial.print(F(x)); delay(2); Serial.println(y); delay(2)
#define P(x) Serial.print(F(x)); delay(2)
void loop()
{
	Serial.println();
	P("Select member: [1] boom1  : ");
	Serial.print(memberBoom1.getServo()->read());
	P(" (");
	Serial.print(memberBoom1.getAngle());
	PLN(")");

	P(".              [2] boom2  : ");
	Serial.print(memberBoom2.getServo()->read());
	P(" (");
	Serial.print(memberBoom2.getAngle());
	PLN(")");

	P(".              [3] turret : ");
	Serial.print(memberTurret.getServo()->read());
	P(" (");
	Serial.print(memberTurret.getAngle());
	PLN(")");

	P(".              [3]   claw : ");
	Serial.print(memberClaw.getServo()->read());
	P(" (");
	Serial.print(memberClaw.getAngle());
	PLN(")");
	
	uint32_t timeout;

	P("Enter member number: ");
	timeout = millis() + 10000;  
	while (!Serial.available())
	{
		if (millis() > timeout)
		{
			timeout = millis() + 10000;
			Serial.println();
			P("Enter member number: ");
		}
	}

	int memberSelection = Serial.parseInt();
	Serial.println(memberSelection);

	RobotArmMember* member;
	switch (memberSelection)
	{
	case 1:
		member = &memberBoom1;
		break;
	case 2:
		member = &memberBoom2;
		break;
	case 3:
		member = &memberTurret;
		break;
	case 4:
		member = &memberClaw;
		break;
	default:
		PLN("Try again.");
		Serial.println();
		Serial.flush();
		return;
	}

	timeout = millis() + 10000;
	P("Enter target angle: ");
	while (!Serial.available())
	{
		if (millis() > timeout)
		{
			timeout = millis() + 10000;
			Serial.println();
			P("... Enter target angle: ");
		}
	}

	int inputAngle = Serial.parseInt();
	Serial.println(inputAngle);

	Serial.print(inputAngle);
	P(" degrees entered. ");

	//member->write(inputAngle);
	member->slow(inputAngle);

	Serial.print(member->getServo()->read());
	PLN(" degrees written.");
	
	P("Sonar measurement: ");
	Serial.print(sonar.getMeasurement());

	PLN2("boom1 height = ", memberBoom1.getPositionVector()->getHeight());
	PLN2("boom2 height = ", memberBoom2.getPositionVector()->getHeight());
	PLN2("turret height= ", memberTurret.getPositionVector()->getHeight());
	PLN2(" claw height = ", memberClaw.getPositionVector()->getHeight());
	PLN2(".     Height = ", state.getPositionVector()->getHeight());

	PLN2("boom1 radius = ", memberBoom1.getPositionVector()->getRadius());
	PLN2("boom2 radius = ", memberBoom2.getPositionVector()->getRadius());
	PLN2(" claw radius = ", memberClaw.getPositionVector()->getRadius());
	PLN2(".     Radius = ", state.getPositionVector()->getRadius());

	PLN2(".     Theta  = ", state.getPositionVector()->getTheta());
}
#endif // SERIAL_CONTROL_MODE

#ifdef AUTONOMOUS_OPERATION		// regular behaviour loop()
void loop()
{
	// Go to middle of paper and calibrate height.
	/* The following should probably be moved to a class function.
	 * RobotArmState or TopoScan? Proabably TopoScan. */
	//memberBoom2.smooth(100);
	//memberBoom1.smooth(115);
	state.goToPos(RobotArmState::NamedPosition::CenterSonar);
	DEBUG1(F("Sonar sensor centered."));
	delay(500);
	uint32_t heightZero = sonar.getMeasurement();
	DEBUG2(F("Height zeroed to h="), heightZero);

	/* The following should probably be moved to TopoScan. */
	for (uint8_t rad = memberBoom1.getMinAngle(); rad < memberBoom1.getMaxAngle(); rad += 5)
	{
		//memberBoom1.smooth(rad);
		//memberTurret.smooth(90);
		DEBUG1(F("Centering sonar."));
		state.goToPos(RobotArmState::NamedPosition::CenterSonar);
		memberBoom1.smooth(rad);

		// zero height with boom2
		int32_t heightDiff = sonar.getMeasurement() - heightZero;
		uint32_t timeout = millis() + 2000;
		while (abs(heightDiff) > 25 && millis() < timeout)
		{
			memberBoom2.smooth((heightDiff > 0) ? -1 : 1);
			heightDiff = sonar.getMeasurement() - heightZero;
		}
		DEBUG2(F("Sonar height adjusted to zero-level. heightDiff="), heightDiff);

		//int32_t timeDiff;
		//do {
		//	timeDiff = micros() - logTime;
		//} while (timeDiff < 0); // wait for log time

		for (uint8_t theta = memberTurret.getMinAngle(); theta < memberTurret.getMaxAngle(); theta += 2)
		{
			memberTurret.smooth(theta); 
			//logSonarDataEverything(sonar, file, state);
			topoScan.logSonarDataEverything(file);
			file.sync();
		}

		for (uint8_t theta = memberTurret.getMaxAngle(); theta > memberTurret.getMinAngle(); theta -= 2)
		{
			memberTurret.smooth(theta);
			//logSonarDataEverything(sonar, file, state);
			topoScan.logSonarDataEverything(file);
			file.sync();
		}

		DEBUG1(F("Completed scan line."));
	}
}
#endif // AUTONOMOUS_OPERATION