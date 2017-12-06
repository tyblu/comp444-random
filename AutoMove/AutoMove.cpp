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
#include "avr/sleep.h"

namespace AutoMove { void shutdown(); }

//#define AUTONOMOUS_OPERATION		// comment out for serial (USB) control mode
#ifndef AUTONOMOUS_OPERATION
#	define SERIAL_CONTROL_MODE
#endif

//#define AutoMove_DEBUG_MODE
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

// Servo and arm construction constants
#define CLAW_INCLINE 0
#define CLAW_LENGTH_MIN 120L		// 87mm + 33mm = 120mm
#define CLAW_LENGTH_ADJ 32L			// swingarm length [mm]
#define CLAW_ANGLE_SCALE 1000L
#define CLAW_ANGLE_OFFSET -20L
#define CLAW_ANGLE_MAX 110
#define CLAW_ANGLE_MIN 40
#define CLAW_ANGLE_SAFE 50

#define BOOM1_LENGTH 140L
#define BOOM1_ANGLE_SCALE -1000L
#define BOOM1_ANGLE_OFFSET 220L		// 130 + 90
#define BOOM1_ANGLE_MAX 180
#define BOOM1_ANGLE_MIN 86
#define BOOM1_ANGLE_SAFE 90

#define BOOM2_LENGTH 145L
#define BOOM2_ANGLE_SCALE 1000L
#define BOOM2_ANGLE_OFFSET -125L
#define BOOM2_ANGLE_MAX 150
#define BOOM2_ANGLE_MIN 20
#define BOOM2_ANGLE_SAFE 95

#define TURRET_ANGLE_SCALE 490L		// 24:49 gear ratio
#define TURRET_ANGLE_OFFSET 0L
#define TURRET_ANGLE_MIN 0
#define TURRET_ANGLE_MAX 180
#define TURRET_ANGLE_SAFE 10

#define PAIR_BOOM2MINS_COUNT 10

/* Important positions determined through manual control and measurement. */
/* PRESET_POSITIONS_COUNT is defined in RobotArmState.h */
PositionVector presetPositions[PRESET_POSITIONS_COUNT] =
{
	{ 100, 265,  63 },		// CenterSonar	- sonar sensor is centered between pylons
	{   0, 156,  50 },		// Center		- horizonal end effector near center of pylons
	{   1, 161,   1 },		// Rest			- good place to shut down, off area
	{ 198, 291,  50 },		// MaxHeight	- around 198
	{ -32, 301,   1 },		// MinHeight	- hits storage box around -32 (off area)
	{  98, 385,  50 },		// MaxRadius	- around 385
	{   0, 143,   1 },		// MinRadius	- around 143; can probably be less with direct servo control
	{  20, 165,   4 },		// Cup			- good place to put the 'drop cup' under
	{ 100, 275,  30 },		// corner A		- above pylon 'A' (opposite power supply)
	{ 100, 381,  53 },		// corner B		- above pylon 'B' (nearest to Arduino)
	{ 100, 309,  81 },		// corner C		- above pylon 'C' (nearest to power supply)
	{ 100, 222,  87 }		// corner D		- above pylon 'D' (nearest to turret base)
};

/* Minimum angles for boom2 with respect to various boom1 angles. */
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

// Sonar and SPI SD card
SonarSensor sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN);
SdFatEX sd;
SdFile topoMapBaseline, topoMapCurrent;
SdFile * files[] = { &topoMapBaseline , &topoMapCurrent };
#define FILES_COUNT 2
TopoScan topoScan(state, sonar);
bool sdCardIsWorking;
#define SPI_SPEED_DIVIDER 2	// for use with SD_SCK_MHZ(F_CPU/SPI_SPEED_DIVIDER), min 2

// Force sensor
ForceSensor sensorL(FORCE_SENSOR_ANALOG_A_PIN, FORCE_SENSORS_POWER_PIN, 10);
ForceSensor sensorR(FORCE_SENSOR_ANALOG_B_PIN, FORCE_SENSORS_POWER_PIN, 10);

void setup()
{
	Serial.begin(9600);
	Serial.println("AutoMove.cpp compiled " __DATE__ " at " __TIME__);
	Serial.println();

	// Servo stuff.
	memberBoom1.setAngleConstants(BOOM1_ANGLE_SCALE, BOOM1_ANGLE_OFFSET);
	memberBoom1.setLimits(BOOM1_ANGLE_MIN, BOOM1_ANGLE_MAX, BOOM1_ANGLE_SAFE);
	DEBUG2(F("memberBoom1.getMinAngle:  "), memberBoom1.getMinAngle());
	DEBUG2(F("memberBoom1.getMaxAngle:  "), memberBoom1.getMaxAngle());

	memberBoom2.setAngleConstants(BOOM2_ANGLE_SCALE, BOOM2_ANGLE_OFFSET);
	memberBoom2.setLimits(BOOM2_ANGLE_MIN, BOOM2_ANGLE_MAX, BOOM2_ANGLE_SAFE);
	DEBUG2(F("memberBoom2.getMinAngle:  "), memberBoom2.getMinAngle());
	DEBUG2(F("memberBoom2.getMaxAngle:  "), memberBoom2.getMaxAngle());

	memberClaw.setAngleConstants(CLAW_ANGLE_SCALE, CLAW_ANGLE_OFFSET);
	memberClaw.setLimits(CLAW_ANGLE_MIN, CLAW_ANGLE_MAX, CLAW_ANGLE_SAFE);
	DEBUG2(F("memberClaw.getMinAngle:   "), memberClaw.getMinAngle());
	DEBUG2(F("memberClaw.getMaxAngle:   "), memberClaw.getMaxAngle());

	memberTurret.setAngleConstants(TURRET_ANGLE_SCALE, TURRET_ANGLE_OFFSET);
	memberTurret.setLimits(TURRET_ANGLE_MIN, TURRET_ANGLE_MAX, TURRET_ANGLE_SAFE);
	DEBUG2(F("memberTurret.getMinAngle: "), memberTurret.getMinAngle());
	DEBUG2(F("memberTurret.getMaxAngle: "), memberTurret.getMaxAngle());

	state.init();
	state.goToPosition(RAS::NamedPosition::Rest);
	state.attach();
	state.servoPowerOn();
	DEBUG1(F("Servos powered on."));

	//state.sweep();
	//state.sweepPresetPositions();
	state.goToPosition(RAS::NamedPosition::Rest);
	state.servoPowerOff();

	// SD Card stuff
	DEBUG1(F("Starting SPI SD card stuff."));
	sdCardIsWorking = AutoMoveSD::startScript(sd, SD_CS_PIN, SPI_SPEED_DIVIDER);
	sdCardIsWorking = AutoMoveSD::initTopo(sd, "/TopoMaps", "base", topoMapBaseline);
	sdCardIsWorking = AutoMoveSD::initTopo(sd, "/TopoMaps", "topo", topoMapCurrent);
	if (sdCardIsWorking)
	{
		topoScan.logSonarDataHeaderEverything(topoMapBaseline);
		topoScan.logSonarDataHeaderEverything(topoMapCurrent);
		Serial.print(F("SD card files initialized: "));
		AutoMoveSD::serialPrintlnFileNames(*files, FILES_COUNT);
	}
	else
		Serial.println(F("SD card malfunction."));

	// Force sensor stuff.
	DEBUG2(F("LHS sensor reading: "), sensorL.read());
	DEBUG2(F("RHS sensor reading: "), sensorR.read());
}

#ifdef SERIAL_CONTROL_MODE			// arm controlled by USB serial commands
#define PLN(x) Serial.println(F(x)); delay(2)
#define PLN2(x,y) Serial.print(F(x)); delay(2); Serial.println(y); delay(2)
#define P(x) Serial.print(F(x)); delay(2)
#define printPos(p) Serial.write('['); Serial.print(p.h); Serial.write(','); Serial.print(p.r); Serial.write(','); Serial.print(p.th); Serial.write(']')
void loop()
{
	if (!state.isServoPowerOn())
		state.servoPowerOn();

	if (sdCardIsWorking)
		topoScan.logSonarData(topoMapCurrent);

	Serial.println();
	PLN("[#] [name] : [servo angle] ([physical angle])");
	P("[1] boom1  : ");
	Serial.print(memberBoom1.getServo()->read());
	P(" (");
	Serial.print(memberBoom1.getPhysicalAngle());
	PLN(")");

	P("[2] boom2  : ");
	Serial.print(memberBoom2.getServo()->read());
	P(" (");
	Serial.print(memberBoom2.getPhysicalAngle());
	PLN(")");

	P("[3] turret : ");
	Serial.print(memberTurret.getServo()->read());
	P(" (");
	Serial.print(memberTurret.getPhysicalAngle());
	PLN(")");

	P("[4]   claw : ");
	Serial.print(memberClaw.getServo()->read());
	P(" (");
	Serial.print(memberClaw.getPhysicalAngle());
	PLN(")");

	P("Current position: ");
	printPosition(*state.getPositionVector());
	Serial.println();
	
	uint32_t timeout;
	int32_t timeLeft;
	uint8_t timeoutCount = 0;
	const uint8_t timeoutCountLimit = 25;

	P("Enter height: ");
	timeout = millis() + 10000;
	while (!Serial.available())
	{
		timeLeft = timeout - millis();
		if (timeLeft < 0)
		{
			timeoutCount++;
			if (timeoutCount > timeoutCountLimit) { AutoMove::shutdown(); }
			timeout = millis() + 10000;
			Serial.println();
			P("Enter height: ");
		}
	}
	int hSelection = Serial.parseInt();
	Serial.println(hSelection);

	P("Enter radius: ");
	timeout = millis() + 10000;
	while (!Serial.available())
	{
		timeLeft = timeout - millis();
		if (timeLeft < 0)
		{
			timeoutCount++;
			if (timeoutCount > timeoutCountLimit) { AutoMove::shutdown(); }
			timeout = millis() + 10000;
			Serial.println();
			P("Enter radius: ");
		}
	}
	int rSelection = Serial.parseInt();
	Serial.println(rSelection);

	P("Enter swing angle: ");
	timeout = millis() + 10000;
	while (!Serial.available())
	{
		timeLeft = timeout - millis();
		if (timeLeft < 0)
		{
			timeoutCount++;
			if (timeoutCount > timeoutCountLimit) { AutoMove::shutdown(); }
			timeout = millis() + 10000;
			Serial.println();
			P("Enter swing angle: ");
		}
	}
	int thSelection = Serial.parseInt();
	Serial.println(thSelection);

	PositionVector posSelection(hSelection, rSelection, thSelection);
	P("Going to position: (h,r,th) = ");
//	printPos(posSelection);
	printPosition(posSelection);	// bare function in RobotArmState.cpp
	Serial.println();
	state.goToPosition(posSelection);

	state.getPositionVector()->updateAll();
	
	PLN2("Sonar measurement: ", sonar.getMeasurement());
	PLN2("Force LHS: ", sensorL.read());
	PLN2("Force RHS: ", sensorR.read());
	P("Current position: ");
	printPosition(*(state.getPositionVector()));
	Serial.println();

//	PLN2("boom1 height = ", memberBoom1.getPositionVector()->getHeight());
//	PLN2("boom2 height = ", memberBoom2.getPositionVector()->getHeight());
//	PLN2("turret height= ", memberTurret.getPositionVector()->getHeight());
//	PLN2(".claw height = ", memberClaw.getPositionVector()->getHeight());
//	PLN2(".     HEIGHT = ", state.getPositionVector()->getHeight());
//
//	PLN2("boom1 radius = ", memberBoom1.getPositionVector()->getRadius());
//	PLN2("boom2 radius = ", memberBoom2.getPositionVector()->getRadius());
//	PLN2(".claw radius = ", memberClaw.getPositionVector()->getRadius());
//	PLN2(".     RADIUS = ", state.getPositionVector()->getRadius());
//
//	PLN2(".     THETA  = ", state.getPositionVector()->getTheta());
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
	for (uint8_t radius = 0; rad < memberBoom1.getMaxAngle(); rad += 5)
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

namespace AutoMove
{
	void shutdown()
	{
		state.goToPosition(RAS::NamedPosition::Rest);
		state.servoPowerOff();

		for (uint8_t i = 0; i < FILES_COUNT; i++)
			files[i]->close();

		Serial.println(F("\n*.*.*.*.* { I'm sleepy. Goodnight! Shutting down... } *.*.*.*.* \n"));
		Serial.flush();

		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		sleep_enable();
		sleep_mode();

		/* Program should never advanced past this point without something to wake it up. */

		sleep_disable();
		state.servoPowerOn();
		DEBUG1(F("Turned back on all by myself! (wtf)"));
	}
}