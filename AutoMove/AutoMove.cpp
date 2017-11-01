#include <Arduino.h>
#include "C:\Users\tyblu\Documents\repos\comp444-random\AutoMove\RobotArmMember.h"
#include "RobotArmState.h"
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
int digitalRead100(uint8_t pin);
class StringPlus : public String {
	StringPlus substring(uint8_t fromIndex);
};

// Servo stuff.
#define SERVO_ON
#ifdef SERVO_ON
#define BOOM1_PWM_PIN 6
#define BOOM2_PWM_PIN 9
#define CLAW_PWM_PIN 5
#define TURRET_PWM_PIN 3
#define SERVO_POWER_CONTROL_PIN 8
//#define SERVO_POWER_FEEDBACK_PIN 1

//class RobotArmStateDefunct		// should put most of these states into RobotArmMember object
//{
//public:
//	struct ServoAngles { uint16_t boom1, boom2, turret, claw; };
//	enum ServoName { Boom1, Boom2, Turret, Claw };
//	enum EndEffectorState { P45Deg, P00Deg, N45Deg, N90Deg, N135Deg };
//
//public:
//
//	RobotArmState(TybluServoList * arg_servos, EndEffectorState state)
//		: list(arg_servos)
//		, endEffectorState(state)
//		, k1(-1)
//		, k2(1)
//		//		, k3(1)
//		, k4(1)
//		, p1(0)
//		, p2(0)
//		//		, p3(0)
//		, p4(0)
//	{
//		angles = list->getServoAngles();
//		setTweaks();	// k and p values
//	}
//
//	RobotArmMember getServo(ServoName servoName)
//	{
//		switch (servoName)
//		{
//		case ServoName::Boom1: return list->boom1;
//		case ServoName::Boom2: return list->boom2;
//		case ServoName::Turret: return list->turret;
//		case ServoName::Claw: return list->claw;
//		default:
//			return;
//		}
//	}
//
//	uint16_t getRadius()
//	{
//		return getR1() + getR2() + getR3();
//	}
//
//	uint16_t getHeight()
//	{
//		return getH1() + getH2() + getH3();
//	}
//
//	uint16_t getTheta()
//	{
//		angles.turret = list->turret.read();
//		return k3 * angles.turret + p3;
//	}
//
//private:
//	TybluServoList * list;
//	EndEffectorState endEffectorState;
//	int16_t h1, h2, h3, r1, r2, r3, k1, k11, k2, k3, k4, p1, p11, p2, p3, p4;
//	ServoAngles angles;
//
//	void setTweaks()
//	{
//		switch (endEffectorState)
//		{
//		case EndEffectorState::P45Deg:
//			p3 = 32 + 140 * cos(deg2rad(45));
//			break;
//		case EndEffectorState::P00Deg:
//			p3 = 32 + 140;
//			break;
//		case EndEffectorState::N45Deg:
//			p3 = 32 + 140 * cos(deg2rad(-45));
//			break;
//		case EndEffectorState::N90Deg:
//			p3 = 32;
//			break;
//		case EndEffectorState::N135Deg:
//			p3 = 32 + 140 * cos(deg2rad(-135));
//			break;
//		default:
//			break;
//		}
//	}
//
//	int16_t getR1()
//	{
//		angles.boom1 = list->boom1.read();
//		return k11 * cos(deg2rad(k1 * angles.boom1 + p1)) + p11;
//	}
//
//	int16_t getR2()
//	{
//		angles.boom2 = list->boom2.read();
//		return k2 * angles.boom2 + p2;
//	}
//
//	int16_t getR3()
//	{
//		angles.claw = list->claw.read();
//		return k4 * angles.claw + p4;
//	}
//
//	int16_t getH1() { return 0; }
//	int16_t getH2() { return 0; }
//	int16_t getH3() { return 0; }
//};

//class TybluServoList
//{
//public:
//	RobotArmMember boom1, boom2, turret, claw;
//
//	TybluServoList(RobotArmMember argBoom1, RobotArmMember argBoom2, RobotArmMember argTurret, RobotArmMember argClaw)
//		: boom1(argBoom1)
//		, boom2(argBoom2)
//		, turret(argTurret)
//		, claw(argClaw)
//	{}
//
//	RobotArmMember current()
//	{
//		switch (iterator)
//		{
//		case 0:	return boom1;
//		case 1: return boom2;
//		case 2: return turret;
//		case 3: return claw;
//		default:
//			iterator = 0;
//			listIteratorHasWrapped = true;
//			return current();
//		}
//	}
//
//	RobotArmMember next()
//	{
//		iterator++;
//		listIteratorHasWrapped = false;
//		return current();
//	}
//
//	bool listFinished()
//	{
//		return listIteratorHasWrapped;
//	}
//
//	void restartList()
//	{
//		listIteratorHasWrapped = false;
//		iterator = 0;
//	}
//
//	RobotArmState::ServoAngles getServoAngles()
//	{
//		return RobotArmState::ServoAngles{ boom1.read(), boom2.read(), turret.read(), claw.read() };
//	}
//
//private:
//	uint8_t iterator = 0;
//	bool listIteratorHasWrapped = false;
//};

// TowerPro 946R, sensor needs verification
RobotArmMember memberBoom1(RobotArmMember::ServoName::Boom1, 
	135, -60, BOOM1_PWM_PIN, 100, 150, 110, A1, 0.557, -49.64);
// Power HD 1501MG
RobotArmMember memberBoom2(RobotArmMember::ServoName::Boom2, 
	142, -90, BOOM2_PWM_PIN, 70, 115, 80, A0, 1.113, -147.1);
// SG90 (micro), not measured -- will probably swap for TowerPro 946R
RobotArmMember memberTurret(RobotArmMember::ServoName::Turret, 
	0, 0, TURRET_PWM_PIN, 30, 150, 90, A3, 1.000, -1.000);
// TowerPro 946R, angles need verification
RobotArmMember memberClaw(RobotArmMember::ServoName::Claw, 
	0, 0, CLAW_PWM_PIN, 60, 130, 100, A2, 0.557, -61.28);

RobotArmState state(RobotArmState::EndEffectorState::P00Deg, memberBoom1, memberBoom2, memberTurret, memberClaw);
#endif // SERVO_ON

// Sonar stuff.
#define SONAR_ON
#ifdef SONAR_ON
#define SONAR_TRIGGER_PIN 4
#define SONAR_ECHO_PIN 7
void logSonarData(SonarSensor & arg_sonar, SdFile & arg_file, RobotArmState& state);
void logSonarDataHeader(SdFile & arg_file);
void logSonarDataEverything(SonarSensor & arg_sonar, SdFile & arg_file, RobotArmState& state);
void logSonarDataHeaderEverything(SdFile & arg_file);
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

// Force sensor stuff.
#define FORCE_SENSORS_ON
#ifdef FORCE_SENSORS_ON
#define FORCE_SENSOR_ANALOG_A_PIN A4
#define FORCE_SENSOR_ANALOG_B_PIN A5
#define FORCE_SENSORS_POWER_PIN 2
#endif

void setup()
{
	Serial.begin(9600);
	Serial.println(__FILE__ " compiled " __DATE__ " at " __TIME__);
	Serial.println();

	// Servo stuff.
#ifdef SERVO_ON
	digitalWrite(SERVO_POWER_CONTROL_PIN, LOW);	// ensure servo power is off
	pinMode(SERVO_POWER_CONTROL_PIN, OUTPUT);
	//pinMode(SERVO_POWER_FEEDBACK_PIN, INPUT);

	while (!state.list.isFinished())
	{
		RobotArmMember servo = state.getServo(state.list.current());
		servo.write(servo.getSafeAngle());
		servo.attach();

		DEBUG3(servo.attached(), F("Servo attached."), F("Servo failed to attach."));

		state.list.next();
	}

	// power on servos
	digitalWrite(SERVO_POWER_CONTROL_PIN, HIGH);
	//do {	// USB programming issue only
	//	DEBUG1("Waiting for user to plug servo feedback wire to pin 0.");
	//	delay(1000);
	//} while (digitalRead100(SERVO_POWER_FEEDBACK_PIN) == LOW);
	//DEBUG3(digitalRead100(SERVO_POWER_FEEDBACK_PIN) == HIGH, "Servos turned on.", "Servos still off.");

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
	
	//logSonarDataHeader(file);
	logSonarDataHeaderEverything(file);
	
	// Start on a multiple of the sample interval.
	logTime = micros() / (1000UL * SAMPLE_INTERVAL_MS) + 1;
	logTime *= 1000UL * SAMPLE_INTERVAL_MS;
#endif // SD_ON
}

void loop()
{
	// SD Card stuff
#ifdef SD_ON
	logTime += 1000UL * SAMPLE_INTERVAL_MS;	// time for next record

	int32_t diff;
	do {
		diff = micros() - logTime;
	} while (diff < 0); // wait for log time

	// Check for data rate too high.
	DEBUG3(diff > 10, F("Data rate too high."), F("Data rate good."));

	//logSonarData(sonar, file, state);
	logSonarDataEverything(sonar, file, state);

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
//	String newFilename = String(ULONG_MAX, DEC);
	String str = "01234567";
	String newFilename = str + ".ext";
	uint8_t fileNumber = 0;

	arg_sd.chdir(true);	// go to root dir
	arg_sd.chdir(folder, true); // go to folder
	do {
		if (fileNumber > 999)
			return;		// quit, too many files
		else if (fileNumber > 100)
			newFilename = "sonar";
		else if (fileNumber > 10)
			newFilename = "sonar0";
		else
			newFilename = "sonar00";

		newFilename.concat(fileNumber++);
		newFilename.concat('.');
		newFilename.concat(extension);
		newFilename.toCharArray(filename, 13);
	} while (arg_sd.exists(filename));
	arg_sd.chdir(true);	// back to root dir
	return;
}

void logSonarData(SonarSensor& arg_sonar, SdFile& arg_file, RobotArmState& state)
{
	arg_file.print(state.getHeight());
	arg_file.write(',');
	arg_file.print(state.getRadius());
	arg_file.write(',');
	arg_file.print(state.getTheta());
	arg_file.write(',');
	arg_file.print(arg_sonar.getMeasurement());
	arg_file.println();
}

void logSonarDataEverything(SonarSensor& arg_sonar, SdFile& arg_file, RobotArmState& state)
{
	arg_file.print(state.getServo(RobotArmMember::ServoName::Boom1).read());
	arg_file.write(',');
	arg_file.print(state.getServo(RobotArmMember::ServoName::Boom1).getAngle());
	arg_file.write(',');
	arg_file.print(state.getServo(RobotArmMember::ServoName::Boom1).getHeight());
	arg_file.write(',');
	arg_file.print(state.getServo(RobotArmMember::ServoName::Boom1).getRadius());
	arg_file.write(',');

	arg_file.print(state.getServo(RobotArmMember::ServoName::Boom2).read());
	arg_file.write(',');
	arg_file.print(state.getServo(RobotArmMember::ServoName::Boom2).getAngle());
	arg_file.write(',');
	arg_file.print(state.getServo(RobotArmMember::ServoName::Boom2).getHeight());
	arg_file.write(',');
	arg_file.print(state.getServo(RobotArmMember::ServoName::Boom2).getRadius());
	arg_file.write(',');

	arg_file.print(state.getServo(RobotArmMember::ServoName::Turret).read());
	arg_file.write(',');
	arg_file.print(state.getServo(RobotArmMember::ServoName::Turret).getAngle());
	arg_file.write(',');
	arg_file.print(state.getServo(RobotArmMember::ServoName::Turret).getHeight());
	arg_file.write(',');
	arg_file.print(state.getServo(RobotArmMember::ServoName::Turret).getRadius());
	arg_file.write(',');

	arg_file.print(state.getServo(RobotArmMember::ServoName::Claw).read());
	arg_file.write(',');
	arg_file.print(state.getServo(RobotArmMember::ServoName::Claw).getAngle());
	arg_file.write(',');
	arg_file.print(state.getServo(RobotArmMember::ServoName::Claw).getHeight());
	arg_file.write(',');
	arg_file.print(state.getServo(RobotArmMember::ServoName::Claw).getRadius());
	arg_file.write(',');

	arg_file.print(state.getHeight());
	arg_file.write(',');
	arg_file.print(state.getRadius());
	arg_file.write(',');
	arg_file.print(state.getTheta());
	arg_file.write(',');

	arg_file.print(arg_sonar.getMeasurement());

	arg_file.println();
}

void logSonarDataHeaderEverything(SdFile& arg_file)
{
	arg_file.print(F("boom1 servo angle, boom1 adj angle, boom1 height, boom1 radius,"));
	arg_file.print(F("boom2 servo angle, boom2 adj angle, boom2 height, boom2 radius,"));
	arg_file.print(F("turrent servo angle, turret adj angle, turret height, turret radius,"));
	arg_file.print(F("claw servo angle, claw adj angle, claw height, claw radius,"));
	arg_file.print(F("height, radius, theta, sonar measurement"));
	arg_file.println();
	DEBUG1(F("Header (everything) printed to SD card... I think."));
}

void logSonarDataHeader(SdFile& arg_file)
{
	arg_file.print(F("height, radius, theta, sonar measurement"));
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

int digitalRead100(uint8_t pin)
{
	uint32_t sum = 0;
	for (uint8_t i = 0; i < 100; i++)
		sum += digitalRead(pin);

	if (sum > 50)
		return HIGH;
	else
		return LOW;
}