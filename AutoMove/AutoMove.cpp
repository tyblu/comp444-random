#include <Arduino.h>
#include "TybluServo.h"
#include "SonarSensor.h"
#include "SdFat.h"
#include "limits.h"

#define AutoMove_DEBUG_MODE
#ifdef AutoMove_DEBUG_MODE
#	define PRE F("AutoMove : ")
#	define DEBUG1(x) Serial.print(PRE); Serial.println(x); delay(2)	// note missing ';'
#	define DEBUG2(x,y) Serial.print(PRE); Serial.print(x); Serial.println(y); delay(2)	// note missing ';'
#	define DEBUG3(f,xT,xF) Serial.print(PRE); if (f) { Serial.println(xT); } else { Serial.println(xF); } delay(2)	// note mising ';'
#else
#	define DEBUG1(x)
#	define DEBUG2(x,y)
#	define DEBUG3(f,xT,xF)
#endif

// Random stuff that should definitely go somewhere else.
float deg2rad(int16_t degrees);
int16_t rad2deg(float radians);

// Servo stuff.
#define SERVO_ON
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
class TybluServoList
{
public:
	TybluServo boom1, boom2, turret, claw;

	TybluServoList(TybluServo argBoom1, TybluServo argBoom2, TybluServo argTurret, TybluServo argClaw)
		: boom1(argBoom1)
		, boom2(argBoom2)
		, turret(argTurret)
		, claw(argClaw)
	{}

	TybluServo current()
	{
		switch (iterator)
		{
		case 0:	return boom1;
		case 1: return boom2;
		case 2: return turret;
		case 3: return claw;
		default:
			iterator = 0;
			listIteratorHasWrapped = true;
			return current();
		}
	}

	TybluServo next()
	{
		iterator++;
		listIteratorHasWrapped = false;
		return current();
	}

	bool listFinished()
	{
		return listIteratorHasWrapped;
	}

	void restartList()
	{
		listIteratorHasWrapped = false;
		iterator = 0;
	}

	RobotArmState::ServoAngles getServoAngles()
	{
		return RobotArmState::ServoAngles{ boom1.read(), boom2.read(), turret.read(), claw.read() };
	}

private:
	uint8_t iterator = 0;
	bool listIteratorHasWrapped = false;
};
TybluServoList robotArmServos(
	TybluServo(BOOM1_PWM_PIN, 100, 150, 110, A1, 0.557, -49.64),	// TowerPro 946R, sensor needs verification
	TybluServo(BOOM2_PWM_PIN, 70, 115, 80, A0, 1.113, -147.1),	// Power HD 1501MG
	TybluServo(TURRET_PWM_PIN, 30, 150, 90, A3, 1.000, -1.000),	// SG90 (micro), not measured
	TybluServo(CLAW_PWM_PIN, 60, 130, 100, A2, 0.557, -61.38) // TowerPro 946R, angles need verification
);

class RobotArmState		// should put most of these states into TybluServo object
{
public:
	struct ServoAngles { uint16_t boom1, boom2, turret, claw; };
	enum ServoName { Boom1, Boom2, Turret, Claw };
	enum EndEffectorState { P45Deg, P00Deg, N45Deg, N90Deg, N135Deg };

	RobotArmState(TybluServoList * arg_servos, EndEffectorState state)
		: list(arg_servos)
		, endEffectorState(state)
		, k1(-1)
		, k2(1)
		//		, k3(1)
		, k4(1)
		, p1(0)
		, p2(0)
		//		, p3(0)
		, p4(0)
	{
		angles = list->getServoAngles();
		setTweaks();	// k and p values
	}

	TybluServo getServo(ServoName servoName)
	{
		switch (servoName)
		{
		case ServoName::Boom1: return list->boom1;
		case ServoName::Boom2: return list->boom2;
		case ServoName::Turret: return list->turret;
		case ServoName::Claw: return list->claw;
		default:
			return;
		}
	}

	uint16_t getRadius()
	{
		return getR1() + getR2() + getR3();
	}

	uint16_t getHeight()
	{
		return getH1() + getH2() + getH3();
	}

	uint16_t getTheta()
	{
		angles.turret = list->turret.read();
		return k3 * angles.turret + p3;
	}

private:
	TybluServoList * list;
	EndEffectorState endEffectorState;
	int16_t h1, h2, h3, r1, r2, r3, k1, k11, k2, k3, k4, p1, p11, p2, p3, p4;
	ServoAngles angles;

	void setTweaks()
	{
		switch (endEffectorState)
		{
		case EndEffectorState::P45Deg:
			p3 = 32 + 140 * cos(deg2rad(45));
			break;
		case EndEffectorState::P00Deg:
			p3 = 32 + 140;
			break;
		case EndEffectorState::N45Deg:
			p3 = 32 + 140 * cos(deg2rad(-45));
			break;
		case EndEffectorState::N90Deg:
			p3 = 32;
			break;
		case EndEffectorState::N135Deg:
			p3 = 32 + 140 * cos(deg2rad(-135));
			break;
		default:
			break;
		}
	}

	int16_t getR1()
	{
		angles.boom1 = list->boom1.read();
		return k11 * cos(deg2rad(k1 * angles.boom1 + p1)) + p11;
	}

	int16_t getR2()
	{
		angles.boom2 = list->boom2.read();
		return k2 * angles.boom2 + p2;
	}

	int16_t getR3()
	{
		angles.claw = list->claw.read();
		return k4 * angles.claw + p4;
	}

	int16_t getH1() { return 0; }
	int16_t getH2() { return 0; }
	int16_t getH3() { return 0; }
};
#endif // SERVO_ON

// Sonar stuff.
#define SONAR_ON
#ifdef SONAR_ON
#define SONAR_TRIGGER_PIN 4
#define SONAR_ECHO_PIN 7
void logSonarData(SonarSensor & arg_sonar, SdFile & arg_file, RobotArmState::ServoAngles pos);
void logSonarDataHeader(SdFile & arg_file);
SonarSensor sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN);
#endif	// SONAR_ON

// SPI SD card stuff.
#define SD_ON
#ifdef SD_ON
#define SD_CS_PIN 10
#define SD_SCK_PIN 13	// ICSP #3
#define SD_MOSI_PIN 11	// ICSP #4
#define SD_MISO_PIN 12	// ICSP #1
void getUniqueShortFileName(char * filename, SdFatEX & arg_sd, const char * folder, const char * extension);
SdFatEX sd;
SdFile file;
const uint32_t SAMPLE_INTERVAL_MS = 500;	// interval between data records [us]
uint32_t logTime;							// time for next data record [us]
#endif // SD_ON

void setup()
{
	Serial.begin(9600);
	Serial.println(__FILE__ " compiled " __DATE__ " at " __TIME__);
	Serial.println();

	// Servo stuff.
#ifdef SERVO_ON
	while (!robotArmServos.listFinished())
	{
		TybluServo servo = robotArmServos.current();
		servo.write(servo.getSafeAngle());
		servo.attach();

		DEBUG3(servo.attached(), F("Servo attached."), F("Servo failed to attach."));

		robotArmServos.next();
	}

	robotArmServos.restartList();
	while (!robotArmServos.listFinished())
	{
		TybluServo servo = robotArmServos.current();
		servo.smooth(servo.getMinAngle());
		servo.smooth(servo.getMaxAngle());
		servo.smooth(servo.getSafeAngle());

		DEBUG1(F("Finished wiggling a servo."));

		robotArmServos.next();
	}

#endif // SERVO_ON

	// SD Card stuff
#ifdef SD_ON
	DEBUG1(F("Starting SPI SD card stuff."));
	/* Initialize at the highest speed supported by the board that is
	// not over 50 MHz. Try a lower speed if SPI errors occur. */
	DEBUG3(!sd.begin(SD_CS_PIN, SD_SCK_MHZ(50)), F("SD init failed."), F("SD init success."));
#ifndef AutoMove_DEBUG_MODE
	if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(50))
		sd.initErrorHalt();
#endif

	if (!sd.exists("TopoMaps"))
	{
		DEBUG3(sd.mkdir("TopoMaps"), F("Create \"TopoMaps\" succeeded."), F("Create \"TopoMaps\" failed."));
#ifndef AutoMove_DEBUG_MODE
		sd.mkdir("TopoMaps");
#endif
	}

	char filename[13];
	getUniqueShortFileName(filename, sd, "/TopoMaps", "txt");
	DEBUG2(F("Unique short file name: "), filename);

#ifdef AutoMove_DEBUG_MODE
	if (!sd.chdir("/TopoMaps")) { DEBUG1(F("Change directory to \"TopoMaps\" failed.")); }
	if (!file.open(filename, O_CREAT | O_WRITE )) { DEBUG2(F("Open file failed: "), filename); }
#else
	sd.chdir("/TopoMaps");
	file.open(filename, O_CREAT);
#endif
	
	logSonarDataHeader(file);
	
	// Start on a multiple of the sample interval.
	logTime = micros() / (1000UL * SAMPLE_INTERVAL_MS) + 1;
	logTime *= 1000UL * SAMPLE_INTERVAL_MS;
#endif // SD_ON
}

void loop()
{
	// SD Card stuff
#ifdef SD_ON

	// Time for next record.
	logTime += 1000UL * SAMPLE_INTERVAL_MS;

	// Wait for log time.
	int32_t diff;
	do {
		diff = micros() - logTime;
	} while (diff < 0);

	// Check for data rate too high.
	DEBUG3(diff > 10, F("Data rate too high."), F("Data rate good."));

	logSonarData(sonar, file, robotArmServos.getServoAngles());

	DEBUG3(!file.sync() || file.getWriteError(), F("Write error."), F("Write success."));
#ifndef AutoMove_DEBUG_MODE
	file.sync();
#endif
#endif // SD_ON
}

/* 
 * Input:
 * char* filename			return filename value, 13-byte char array (incl \0)
 *		Includes 8 numerical characters for file name, 3 character extension,
 *		and 2 characters for period (extension separator) and null terminator.
 *							
 * SdFatEX& arg_sd			FAT volume (etc) object
 * const char * folder		folder to compare for uniqueness
 * const char * extension	filename extension, 4-byte char array (incl \0)
 *
 * Post-conditions: char * filename changed to unique filename.
 *	arg_sd working directory changed to volume root directory.
 */
void getUniqueShortFileName(char * filename, SdFatEX & arg_sd, const char * folder, const char * extension)
{
	String newFilename = String(ULONG_MAX, DEC);

	arg_sd.chdir(true);	// go to root dir
	arg_sd.chdir(folder, true); // go to folder
	do {
		newFilename = newFilename.substring(newFilename.length() - String("01234567").length());
		newFilename.concat('.');
		newFilename.concat(extension);
		newFilename.toCharArray(filename, 13);
	} while (arg_sd.exists(filename));
	arg_sd.chdir(true);	// back to root dir
	return;
}

void logSonarData(SonarSensor & arg_sonar, SdFile & arg_file, RobotArmState::ServoAngles pos)
{
	arg_file.print(pos.boom1);
	arg_file.print(',');
	arg_file.print(pos.boom2);
	arg_file.print(',');
	arg_file.print(pos.turret);
	arg_file.print(',');
	arg_file.print(pos.claw);
	arg_file.print(',');
	arg_file.print(arg_sonar.getMeasurement());
	arg_file.println();
}

void logSonarDataHeader(SdFile & arg_file)
{
	arg_file.print(F("boom1,boom2,turret,claw,sonar"));
	arg_file.println();
	DEBUG1(F("Header printed to SD card... I think."));
}

float deg2rad(int16_t degrees)
{
	return degrees * PI / (float)180;
}

int16_t rad2deg(float radians)
{
	return (int16_t)(radians * 180 / PI);
}