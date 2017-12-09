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
#include "RollingAverage.h"

namespace AutoMove // signatures only
{ 
	void shutdown();											// closest thing to powering off
	bool scanForObject(int32_t timeout, uint32_t baseline = 0);	// look for objects from up on high
	void gripObjectAt(int h, int r, int th);					// grip object at (h,r,th)
	void releaseObjectAt(int h, int r, int th);					// release object at (h,r,th)
	void releaseObject();										// release at current position
}	

#define AUTONOMOUS_OPERATION		// comment out for serial (USB) control mode
#ifndef AUTONOMOUS_OPERATION
#	define SERIAL_CONTROL_MODE
#endif

#define SCAN_OBJECT_COUNT 100
#define SCAN_OBJECT_DATA_POINTS 3
#define SCAN_OBJECT_HYSTERISIS 2000UL

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

#define FORCE_SENSOR_MIN 41		// about 200mV
#define FORCE_SENSOR_MAX 982	// about 5V - 200mV = 4.8V
#define GRIP_FORCE_MAX 500

#define PLATFORM_HEIGHT -5

/* Important positions determined through manual control and measurement. */
/* PRESET_POSITIONS_COUNT is defined in RobotArmState.h */
PositionVector presetPositions[PRESET_POSITIONS_COUNT] =
{
	{ 180, 280,  63 },		// CenterSonar	- sonar sensor is centered between pylons
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
ForceSensor sensorL(FORCE_SENSOR_ANALOG_L_PIN, FORCE_SENSORS_POWER_PIN, FORCE_SENSOR_MIN, FORCE_SENSOR_MAX);
ForceSensor sensorR(FORCE_SENSOR_ANALOG_R_PIN, FORCE_SENSORS_POWER_PIN, FORCE_SENSOR_MIN, FORCE_SENSOR_MAX);

void setup()
{
	Serial.begin(9600);
	Serial.println("AutoMove.cpp compiled " __DATE__ " at " __TIME__ "\n");

	// Servo stuff.
	memberBoom1.setAngleConstants(BOOM1_ANGLE_SCALE, BOOM1_ANGLE_OFFSET);
	memberBoom1.setLimits(BOOM1_ANGLE_MIN, BOOM1_ANGLE_MAX, BOOM1_ANGLE_SAFE);
	memberBoom2.setAngleConstants(BOOM2_ANGLE_SCALE, BOOM2_ANGLE_OFFSET);
	memberBoom2.setLimits(BOOM2_ANGLE_MIN, BOOM2_ANGLE_MAX, BOOM2_ANGLE_SAFE);
	memberClaw.setAngleConstants(CLAW_ANGLE_SCALE, CLAW_ANGLE_OFFSET);
	memberClaw.setLimits(CLAW_ANGLE_MIN, CLAW_ANGLE_MAX, CLAW_ANGLE_SAFE);
	memberTurret.setAngleConstants(TURRET_ANGLE_SCALE, TURRET_ANGLE_OFFSET);
	memberTurret.setLimits(TURRET_ANGLE_MIN, TURRET_ANGLE_MAX, TURRET_ANGLE_SAFE);

	state.init();
	state.goToPosition(RAS::NamedPosition::CenterSonar);
	state.attach();
	state.servoPowerOn();
	DEBUG1(F("Servos powered on."));

	//state.sweep();
	//state.sweepPresetPositions();
	state.goToPosition(RAS::NamedPosition::CenterSonar);
	state.servoPowerOff();

	// SD Card stuff
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
	Serial.print(F("Force sensors configured (normally off): "));
	ForceSensor::report(sensorL, sensorR, true);
}

#ifdef SERIAL_CONTROL_MODE			// arm controlled by USB serial commands
#define PLN(x) Serial.println(F(x)); delay(2)
#define PLN2(x,y) Serial.print(F(x)); delay(2); Serial.println(y); delay(2)
#define P(x) Serial.print(F(x)); delay(2)
#define printPos(p) Serial.write('['); Serial.print(p.h); Serial.write(','); Serial.print(p.r); Serial.write(','); Serial.print(p.th); Serial.write(']')
void loop()
{
	if (!state.isServoPowerOn())
	{
		state.servoPowerOn();
		state.attach();
		state.goToPosition(RobotArmState::NamedPosition::CenterSonar);
		Serial.println(F("Ready for sonar readings."));
	}

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
	if (!state.isServoPowerOn())
	{
		state.servoPowerOn();
		state.attach();
		state.goToPosition(RobotArmState::NamedPosition::CenterSonar);
		Serial.println(F("Ready to rock and roll!"));
	}

	if (AutoMove::scanForObject(10000))
	{
		DEBUG1(F("Object found, moving it off the platform..."));
		/* presetPosition[9] is position 'B', the platform corner nearest to the Arduino. */
		AutoMove::gripObjectAt(PLATFORM_HEIGHT + 5, presetPositions[9].r, presetPositions[9].th);
		delay(500);
		state.goToPosition(RobotArmState::NamedPosition::Cup);
		AutoMove::releaseObject();
		delay(500);
	}
	else
	{
		DEBUG1(F("Did not detect object. Continuing to scan..."));
		delay(500);
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

	bool scanForObject()
	{
		return scanForObject(5000L, 0UL);	// TODO: make the 5000UL a #define
	}

	bool scanForObject(int32_t timeout, uint32_t baseline = 0)
	{
		if (!state.isServoPowerOn())
		{
			state.servoPowerOn();
			state.attach();
		}

		state.goToPosition(RobotArmState::NamedPosition::CenterSonar);
		delay(250);	// wait to stabilize	TODO: make 250 a #define

		/* Get baseline sonar measurement if not provided. */
		if (baseline = 0)
			baseline = sonar.getMeasurement(SCAN_OBJECT_COUNT);

		RollingAverage sonarData(baseline);
		uint32_t timeEnd = millis() + timeout;
		while (timeout > 0)
		{
			sonarData.push(sonar.getMeasurement(SCAN_OBJECT_COUNT));

			/* If an object was on the table while doing baseline scan, get new baseline. */
			if (sonarData.peek() + SCAN_OBJECT_HYSTERISIS < baseline)
			{
				baseline = sonar.getMeasurement(SCAN_OBJECT_COUNT);
				sonarData.set(baseline);
			}

			if (sonarData.getAverage() > baseline + SCAN_OBJECT_HYSTERISIS 
				|| sonarData.getStdDev() > SCAN_OBJECT_HYSTERISIS)
				return true;

			timeout = timeEnd - millis();
		}
		return false;	// no object detected within time limit
	}

	void gripObjectAt(int h, int r, int th)
	{
		memberClaw.fast(memberClaw.getMinAngle());

		state.goToPosition(
			state.getPositionVector()->getHeight(), 
			r - memberClaw.getPositionVector()->getRadius(), 
			th
		);

		state.goToPosition(
			h,
			r - memberClaw.getPositionVector()->getRadius(),
			th
		);

		state.goToPosition(
			h, 
			r - memberClaw.getPositionVector()->getRadius(memberClaw.toPhysicalAngle(memberClaw.getMaxAngle())) * 2 / 3,
			th
		);

		memberClaw.change(20);

		state.goToPosition(
			h,
			r - memberClaw.getPositionVector()->getRadius(memberClaw.toPhysicalAngle(memberClaw.getMaxAngle())) * 1 / 3,
			th
		);

		memberClaw.change(20);

		while (sensorL.read() + sensorR.read() < 2 * GRIP_FORCE_MAX 
			&& memberClaw.getServo()->read() < memberClaw.getMaxAngle())
		{
			state.goToPosition(h, r, th);
			memberClaw.change(2);
		}
	}

	void releaseObjectAt(int h, int r, int th)
	{
		while (sensorL.read() + sensorR.read() > 2 * FORCE_SENSOR_MIN
			&& memberClaw.getServo()->read() > memberClaw.getMinAngle())
		{
			state.goToPosition(h, r, th);
			memberClaw.change(-2);
		}

		memberClaw.fast(memberClaw.getMinAngle());
	}

	void releaseObject()
	{
		releaseObjectAt(
			state.getPositionVector()->h,
			state.getPositionVector()->r,
			state.getPositionVector()->th
		);
	}
}