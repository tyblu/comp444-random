/*
 * RobotArmMember.cpp
 *
 *  Created on: Oct 14, 2017
 *      Author: Tyler Lucas
 */

#include <inttypes.h>
#include "C:\Users\tyblu\Documents\repos\comp444-random\TybluLsq\TybluLsq.h"
#include "C:\Users\tyblu\Documents\repos\QuickStats\QuickStats.h"
#include "IntegerGeometry.h"
#include "C:\Users\tyblu\Documents\repos\comp444-random\AutoMove\RobotArmMember.h"

//#define RobotArmMember_DEBUG_MODE
#ifdef RobotArmMember_DEBUG_MODE
#	define DEBUG1(x) Serial.print("RobotArmMember : "); Serial.println(x); delay(2)	// note missing ';'
#	define DEBUG2(x,y) Serial.print("RobotArmMember : "); Serial.print(x); Serial.println(y); delay(2)	// note missing ';'
#else
#	define DEBUG1(x)
#	define DEBUG2(x,y)
#endif

/* This should probably be put somewhere else. */
template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

// used in getAnalogAngle, calibrateSensor, and smooth
#define MODE_LOWER_LIMIT 10		// 5V *  10/1024 = 49mV
#define MODE_UPPER_LIMIT 512	// 5V * 512/1024 = 2.5V, max in equal voltage divider
#define MODE_EPSILON 1.0
#define CALIB_STEP_SIZE 3
#define CALIB_STEPS 12
#define CALIB_STEP_DELAY 25
#define CALIB_MAX_ITERATIONS 500
#define SMOOTH_ANGLE_MIN 0
#define SMOOTH_ANGLE_MAX 180
#define SMOOTH_ADJUSTMENT_ANGLE 2
#define SMOOTH_ADJUSTMENT_ANGLE_STOPPED 1
#define SMOOTH_ADJUSTMENT_DELAY 25
#define SMOOTH_TIMEOUT_MS 1000		// timeout for changing angle
#define MEASUREMENTS_COUNT 40
#define ANALOG_RAW_TIMEOUT_MS 100	// 100ms timeout for getting analog angle
#define SLOW_TIMEOUT_MS 15

/*
 * Constructor arguments:
 * RobotArmMember::Name name			(enum) Boom1, Boom2, Turret, Claw
 * uint16_t length							Distance between axes.
 * int offset								Difference between physical angle 
											with horizontal plane and servo 
											input angle.
 * int pwmPin								Pin used to control servo.
 * int minAngle, int maxAngle 				Allowed range of movement.
 * int safeAngle							Nominal position for servo.
 * int sensorPin 							Angle sensor pin (A0, A1, ...).
 * float sensorSlope, float sensorOffset	Initial angle sensor linear 
											coefficients.
 */
RobotArmMember::RobotArmMember(Name name, uint16_t length,
	int pwmPin, int min, int max, int safe,
	int sensorPin)
	: Servo()
	, name(name), length(length), pwmPin(pwmPin)
	, minAngle(min), maxAngle(max), safeAngle(safe)
	, sensorPin(sensorPin)
	, angleOffset(0), angleScale1000(1000)
	, sensorOffset(0), sensorScale1000(1000)
	, pos(*this)
{}

/**
 * Pre-conditions: Servo must be attached, have max/min angles set, and have valid
 * sensor pin.
 * Post-conditions: In addition to preconditions, sensor slope and offset variables
 * are set, allowing for accurate returns from RobotArmMember::getAnalogAngle.
 *
 * Returns true if it completes successfully, false otherwise. No variables changed.
 *
 * Calibrates servo position sensor by fitting line to measured results while moving
 * between angleA and angleB. Movements starts at angleA, travels to angleB, back to
 * angleA, and it may repeat this several more times. The servo always obeys the
 * minAngle and maxAngle limits, so make sure angleA and angleB are within them or
 * the calibration will not work (returns false).
 */
bool RobotArmMember::calibrateSensor(int angleA, int angleB)
{
	if (!this->attached())
		return false;

	{	// ensure angleA <= angleB
		int temp1 = min(angleA, angleB);
		angleB = max(angleA, angleB);
		angleA = temp1;
	}

	// invalid sensor pin or calibration test range outside of servo min/max
	if (sensorPin < 0 || angleA < minAngle || angleB > maxAngle)
		return false;

	this->smooth(angleA);
	delay(500);				// wait for servo to reach angleA

	// measure data
	float angles[CALIB_STEPS];
	float measurements[CALIB_STEPS];
	angles[0] = angleA;
	int direction = 1;
	unsigned int iterator = 0;
	unsigned int total_iterations = 0;

	DEBUG1(F("starting do-while..."));

	do {
		total_iterations++;
		// Serial.println(angles[iterator]);
		this->smooth((int)angles[iterator]);
		delay(CALIB_STEP_DELAY);
		measurements[iterator] = this->getAnalogRaw();

//		Serial.println(); Serial.print(iterator); Serial.print(" : "); Serial.print(angles[iterator]); Serial.print(" | "); Serial.print(measurements[iterator]);

		// skips bad data by not advancing iterator, leading to overwrite
		if (measurements[iterator] > MODE_LOWER_LIMIT
				&& measurements[iterator] < MODE_UPPER_LIMIT)
			iterator++;

		// don't try to get angles[-1], below
		if (iterator == 0)
			continue;

		if (angles[iterator-1] <= angleA)
			direction = 1;
		else if (angles[iterator-1] >= angleB)
			direction = -1;

		angles[iterator] = angles[iterator-1] + direction * CALIB_STEP_SIZE;

	} while (iterator < CALIB_STEPS && total_iterations < CALIB_MAX_ITERATIONS);

	DEBUG1(F("finished do-while loop"));

	if (total_iterations >= CALIB_MAX_ITERATIONS)
		Serial.println(F("CALIB_MAX_ITERATIONS reached!"));

	// least squares fit
	float a, b;

	// void TybluLsq::llsq( int n, float x[], float y[], float &a, float &b )
	TybluLsq::llsq(CALIB_STEPS, measurements, angles, a, b);

	this->sensorScale1000 = (long)( a * 1000 );
	this->sensorOffset = (long)b;

	DEBUG2(F("a="), a);
	DEBUG2(F("sensorScale1000="), sensorScale1000);
	DEBUG2(F("b="), b);
	DEBUG2(F("sensorOffset="), sensorOffset);

	return true;
}

/*
 * Same as calibrateSensor(int angleA, int angleB), using max/min angles.
 */
bool RobotArmMember::calibrateSensor()
{
	return calibrateSensor(this->minAngle, this->maxAngle);
}

//void RobotArmMember::setName(Name name)
//{
//	this->name = name;
//}

void RobotArmMember::setMinServoAngle(int minAngle)
{
	this->minAngle = minAngle;
}

void RobotArmMember::setMaxServoAngle(int maxAngle)
{
	this->maxAngle = maxAngle;
}

void RobotArmMember::setSafeServoAngle(int safeAngle)
{
	this->safeAngle = safeAngle;
}

//void RobotArmMember::setSensorPin(int sensorPin)
//{
//	this->sensorPin = sensorPin;
//}

void RobotArmMember::setSensorConstants(long sensorScale1000, long sensorOffset)
{
	this->sensorScale1000 = sensorScale1000;
	this->sensorOffset = sensorOffset;
}

void RobotArmMember::setAngleConstants(long angleScale1000, long angleOffset)
{
	this->angleScale1000 = angleScale1000;
	this->angleOffset = angleOffset;
}

RobotArmMember::Name RobotArmMember::getName()
{
	return this->name;
}

int RobotArmMember::getAngle()
{
	return servoAngleToTrueAngle(this->read());
}

int RobotArmMember::getHeight(int angle)
{
	long height = this->length;
	height *= IntegerGeometry::sin1000(angle);
	height = (height + 500L) / 1000L;	// int division with rounding to nearest
	return height;
}

int RobotArmMember::getHeight()
{
	return getHeight(this->getAngle());
}

int RobotArmMember::getRadius(int angle)
{
	long radius = this->length;
	radius *= IntegerGeometry::cos1000(angle);
	radius = (radius + 500L) / 1000L;	// int division with rounding to nearest
	return radius;
}

int RobotArmMember::getRadius()
{
	return getRadius(this->getAngle());
}

int RobotArmMember::getMinServoAngle()
{
	return this->minAngle;
}

int RobotArmMember::getMinAngle()
{
	return servoAngleToTrueAngle(getMinServoAngle());
}

int RobotArmMember::getMaxServoAngle()
{
	return this->maxAngle;
}

int RobotArmMember::getMaxAngle()
{
	return servoAngleToTrueAngle(getMaxServoAngle());
}

int RobotArmMember::getSafeServoAngle()
{
	return this->safeAngle;
}

int RobotArmMember::getSensorPin()
{
	return this->sensorPin;
}

uint16_t RobotArmMember::getLength()
{
	return this->length;
}

int RobotArmMember::getAnalogAngle()
{
		return this->getAnalogRaw() * sensorScale1000 / 1000 + sensorOffset;
}

int RobotArmMember::getAnalogRaw()
{
	float measurements[MEASUREMENTS_COUNT];
	float stDev = analogDeviationLimit + 1.0;

	unsigned long timeout = millis() + ANALOG_RAW_TIMEOUT_MS;

	while ( stDev > analogDeviationLimit)
	{
		for (int i=0; i<MEASUREMENTS_COUNT; i++)
			measurements[i] = analogRead(sensorPin);
		qs.bubbleSort(measurements, MEASUREMENTS_COUNT);
		stDev = qs.stdev(measurements, MEASUREMENTS_COUNT);

		if (millis() > timeout)
			return 0;	// ruf, ruf, watchdog chases you out!
	}

	float mode = qs.mode(measurements, MEASUREMENTS_COUNT, MODE_EPSILON);

	if (mode < MODE_LOWER_LIMIT || mode > MODE_UPPER_LIMIT)
		return 0;
	else
		return (int)mode;
}

/*
 * If value is < 200 its treated as an angle, otherwise as pulse width in
 * microseconds. If value is < minAngle then it's set to minAngle; if > maxAngle and
 * < 200 then it's set to maxAngle.
 */
void RobotArmMember::write(int value)
{
	if (value < minAngle)
		Servo::write(minAngle);
	else if (value > maxAngle && value < 200)
		Servo::write(maxAngle);
	else							// minAngle > value > maxAngle || value > 200
		Servo::write(value);
}

void RobotArmMember::slow(int value)
{
	if (value < 0 || value > 180)
	{
		DEBUG2(F("Bad angle input: "), value);
		return;
	}

	int currentAngle = this->read();
	int delta = value - currentAngle;
	DEBUG2(F("delta="), delta);
	uint32_t writeTimeout;
	while (delta != 0)
	{
		if (delta > 0)
			currentAngle++;
		else
			currentAngle--;

		this->write(currentAngle);
		delta = value - currentAngle;
		DEBUG2(F("Moved to "), currentAngle);

		writeTimeout = millis() + SLOW_TIMEOUT_MS;
		while (millis() < writeTimeout) {}
	}
}

///*
// * Attempts to pre-set servo position to avoid initial jolts, than attaches.
// * Servo::attach(int) description:
// * 	Attach the given pin to the next free channel, sets pinMode, returns channel
// * 	number or 0 if failure.
// */
//uint8_t RobotArmMember::attach(int pin)
//{
//	this->pwmPin = pin;
//	int currentAnalogAngle = getAnalogAngle();
//	if ( currentAnalogAngle >= minAngle && currentAnalogAngle <= maxAngle )
//		write(currentAnalogAngle);
//	return Servo::attach(pin);
//}

/*
 * Preconditions: pwmPin must have been set either in the [full] constructor or
 * RobotArmMember::attach(int pin) (presumably later followed by a 'detach()).
 *
 * Attaches to previously set pwmPin, calling RobotArmMember::attach(pwmPin). Returns
 * uint8_t 0 on fail (i.e. bad pwmPin), channel number otherwise.
 */
uint8_t RobotArmMember::attach()
{
	return Servo::attach(this->pwmPin);
}

void RobotArmMember::smooth(int targetAngle)
{
	int currentAngle = this->read();

	if (currentAngle < SMOOTH_ANGLE_MIN || currentAngle > SMOOTH_ANGLE_MAX)
	{
		this->write(targetAngle);		// can't figure out where servo is, just write angle
		return;
	}

	bool isMoving = false;
	int lastCurrentAngle = currentAngle;
	unsigned long timeout = millis() + SMOOTH_TIMEOUT_MS;
	while (currentAngle != targetAngle)
	{
//		Serial.print("current="); Serial.print(currentAngle); Serial.print(", target="); Serial.print(targetAngle); Serial.print(", dir="); Serial.println(sgn(targetAngle - currentAngle));
		if ( !isMoving || abs(targetAngle - currentAngle) <= SMOOTH_ADJUSTMENT_ANGLE )
			currentAngle += sgn(targetAngle - currentAngle) * SMOOTH_ADJUSTMENT_ANGLE_STOPPED;
		else
			currentAngle += sgn(targetAngle - currentAngle) * SMOOTH_ADJUSTMENT_ANGLE;

		if (lastCurrentAngle == currentAngle)
			break;

		this->write(currentAngle);
		delay(SMOOTH_ADJUSTMENT_DELAY);
		isMoving = true;

		if (millis() > timeout)	// took too long, probably stuck for some reason
		{
			this->write(targetAngle);
			return;
		}
	}
}

void RobotArmMember::sweep()
{
	int initialAngle = this->read();
	this->smooth(minAngle);
	this->smooth(maxAngle);
	this->smooth(initialAngle);
}

void RobotArmMember::getNameStr(char nameStr[7])
{
	switch (this->name)
	{
	case Name::Boom1:
		strcpy_P(nameStr, servoStr[0]);
		break;
	case Name::Boom2:
		strcpy_P(nameStr, servoStr[1]);
		break;
	case Name::Turret:
		strcpy_P(nameStr, servoStr[2]);
		break;
	case Name::Claw:
		strcpy_P(nameStr, servoStr[3]);
		break;
	default:
		strcpy_P(nameStr, servoStr[4]);
		break;
	}
}

int RobotArmMember::servoAngleToTrueAngle(int angle)
{
	long angleL = (long)angle;
	angleL *= this->angleScale1000;
	angleL = (angleL + 500L) / 1000L;	// int division with rounding to nearest
	angleL += this->angleOffset;
	return (int)angleL;
}

int RobotArmMember::trueAngleToServoAngle(int angle)
{
	long angleL = (long)angle;
	angleL -= this->angleOffset;
	angleL = (angleL * 1000L) - 500L;
	angleL = (angleL + this->angleScale1000 / 2) / this->angleScale1000;
	return (int)angleL;
}


PositionVector::PositionVector(RobotArmMember& member)
	: member(member)
{
	this->length = member.getLength();
	this->name = member.getName();
	
	switch (name)
	{
	case RobotArmMember::Name::Turret:
		this->angle = 0;
		this->radius = 0;
		this->height = 0;
		break;
	case RobotArmMember::Name::Claw:
		this->theta = 0;
		this->height = member.getHeight();	// constant for each claw position
		break;
	default:
		break;
	}
	
	update();
}

void PositionVector::update()
{
	switch (name)
	{
	case RobotArmMember::Name::Turret:
		this->theta = member.getAngle();
		break;
	case RobotArmMember::Name::Claw:
		this->angle = member.getAngle();
		this->radius = member.getRadius();
		break;
	default:	// Boom1 and Boom2
		this->angle = member.getAngle();
		this->radius = member.getRadius();
		this->height = member.getHeight();
		break;
	}
}

int PositionVector::getHeight()
{
	return this->height;
}

int PositionVector::getHeight(int angle)
{
	long heightL = (long)this->length;
	heightL = heightL * IntegerGeometry::sin1000(angle);
	heightL = heightL / 1000L;
	return (int)heightL;
}

int PositionVector::getRadius()
{
	return this->radius;
}

int PositionVector::getRadius(int angle)
{
	switch (name)
	{
	case RobotArmMember::Name::Claw:
		// claw radius
		return 1;
	default:
		long radiusL = (long)this->length;
		radiusL = radiusL * IntegerGeometry::cos1000(angle);
		radiusL = radiusL / 1000L;
		return (int)radiusL;
	}
}

int PositionVector::getTheta()
{
	return this->theta;
}
