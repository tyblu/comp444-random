#include <Arduino.h>
#include "inttypes.h"
#include "C:\Users\tyblu\Documents\repos\comp444-random\AutoMove\RobotArmMember.h"
#include "RobotArmState.h"
#include "SonarSensor.h"
#include "SdFat.h"
#include "limits.h"
#include "ForceSensor.h"

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

// Servo stuff.
#define SERVO_ON
#ifdef SERVO_ON
#define BOOM1_PWM_PIN 6
#define BOOM2_PWM_PIN 9
#define CLAW_PWM_PIN 5
#define TURRET_PWM_PIN 3
#define SERVO_POWER_CONTROL_PIN 8
//#define SERVO_POWER_FEEDBACK_PIN 1

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

RobotArmState state(RobotArmState::EndEffectorState::P00Deg, memberBoom1, 
	memberBoom2, memberTurret, memberClaw);
#endif // SERVO_ON

// Sonar stuff.
#define SONAR_ON
#ifdef SONAR_ON
#define SONAR_TRIGGER_PIN 4
#define SONAR_ECHO_PIN 7
void logSonarData(SonarSensor & arg_sonar, SdFile & arg_file, 
	RobotArmState& state);
void logSonarDataHeader(SdFile & arg_file);
void logSonarDataEverything(SonarSensor & arg_sonar, SdFile & arg_file, 
	RobotArmState& state);
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
void getUniqueShortFileName(char * filename, SdFatEX & arg_sd, 
	const char * folder, const char * extension);
SdFatEX sd;
SdFile file;
#endif // SD_ON

// Force sensor stuff.
#define FORCE_SENSORS_ON
#ifdef FORCE_SENSORS_ON
#define FORCE_SENSOR_ANALOG_A_PIN A4
#define FORCE_SENSOR_ANALOG_B_PIN A5
#define FORCE_SENSORS_POWER_PIN 2
ForceSensor sensorL(FORCE_SENSOR_ANALOG_A_PIN, FORCE_SENSORS_POWER_PIN, 10);
ForceSensor sensorR(FORCE_SENSOR_ANALOG_B_PIN, FORCE_SENSORS_POWER_PIN, 10);
#endif // FORCE_SENSORS_ON

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
	DEBUG3(!sd.begin(SD_CS_PIN, SD_SCK_MHZ(50)), F("begin failed."), F("begin success."));
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
#endif // SD_ON

	// Force sensor stuff.
#ifdef FORCE_SENSORS_ON
	DEBUG2("LHS sensor reading: ", sensorL.read());
	DEBUG2("RHS sensor reading: ", sensorR.read());
#endif FORCE_SENSORS_ON
}

void loop()
{
	// Go to middle of paper and calibrate height.
	memberBoom2.smooth(100);
	memberBoom1.smooth(115);
	delay(500);
	uint32_t heightZero = sonar.getMeasurement();

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
			logSonarDataEverything(sonar, file, state);
			file.sync();
		}

		for (uint8_t theta = memberTurret.getMaxAngle(); theta > memberTurret.getMinAngle(); theta -= 5)
		{
			memberTurret.smooth(theta);
			logSonarDataEverything(sonar, file, state);
			file.sync();
		}
	}
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

// naive debouncing
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