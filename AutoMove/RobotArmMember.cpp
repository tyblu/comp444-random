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
#define SMOOTH_ANGLE_MIN 1
#define SMOOTH_ANGLE_MAX 179
#define SMOOTH_ADJUSTMENT_ANGLE 2
#define SMOOTH_ADJUSTMENT_ANGLE_STOPPED 1
#define SMOOTH_ADJUSTMENT_DELAY 25
#define SMOOTH_TIMEOUT_MS 1000		// timeout for changing angle
#define MEASUREMENTS_COUNT 40
#define ANALOG_RAW_TIMEOUT_MS 100	// 100ms timeout for getting analog angle
#define SLOW_TIMEOUT_MS 15

/*
 * Constructor arguments:
 * int pwmPin								Pin used to control servo.
 * int minAngle, int maxAngle 				Allowed range of movement.
 * int safeAngle							Nominal position for servo.
 * int sensorPin 							Angle sensor pin (A0, A1, ...).
 * float sensorSlope, float sensorOffset	Initial angle sensor linear coefficients.
 */
RobotArmMember::RobotArmMember(ServoName name, uint16_t length, int angleOffset,
	int pwmPin, int min, int max, int safe,
	int sensorPin, float sensorSlope, float sensorOffset)
	: Servo()
	, name(name), length(length), angleOffset(angleOffset), pwmPin(pwmPin)
	, minAngle(min), maxAngle(max), safeAngle(safe)
	, sensorPin(sensorPin), sensorLine(sensorSlope, sensorOffset)
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

	DEBUG1("starting do-while...");

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

	DEBUG1("finished do-while loop");

	if (total_iterations >= CALIB_MAX_ITERATIONS)
		Serial.println("CALIB_MAX_ITERATIONS reached!");

	// least squares fit
	float a, b;

	// void TybluLsq::llsq( int n, float x[], float y[], float &a, float &b )
	TybluLsq::llsq(CALIB_STEPS, measurements, angles, a, b);

	this->sensorLine.m = a;
	this->sensorLine.b = b;

	DEBUG2("a=", a);
	DEBUG2("sensorSlope=", sensorLine.m);
	DEBUG2("b=", b);
	DEBUG2("sensorOffset=", sensorLine.b);

	return true;
}

/*
 * Same as calibrateSensor(int angleA, int angleB), using max/min angles.
 */
bool RobotArmMember::calibrateSensor()
{
	return calibrateSensor(this->minAngle, this->maxAngle);
}

void RobotArmMember::setName(ServoName name)
{
	this->name = name;
}

void RobotArmMember::setAngleOffset(int angleOffset)
{
	this->angleOffset = angleOffset;
}

void RobotArmMember::setMinAngle(int minAngle)
{
	this->minAngle = minAngle;
}

void RobotArmMember::setMaxAngle(int maxAngle)
{
	this->maxAngle = maxAngle;
}

void RobotArmMember::setSafeAngle(int safeAngle)
{
	this->safeAngle = safeAngle;
}

void RobotArmMember::setSensorPin(int sensorPin)
{
	this->sensorPin = sensorPin;
}

void RobotArmMember::setSensorConstants(float slope, float offset)
{
	this->sensorLine.m = slope;
	this->sensorLine.b = offset;
}

RobotArmMember::ServoName RobotArmMember::getName()
{
	return this->name;
}

int RobotArmMember::getAngleOffset()
{
	return this->angleOffset;
}

int RobotArmMember::getAngle()
{
	return this->read() + angleOffset;
}

int RobotArmMember::getHeight()
{
	return IntegerGeometry::intDiv(this->length * IntegerGeometry::bigSin(this->getAngle()), 1000);
}

int RobotArmMember::getRadius()
{
	return IntegerGeometry::intDiv(this->length * IntegerGeometry::bigCos(this->getAngle()), 1000);
}

int RobotArmMember::getMinAngle()
{
	return this->minAngle;
}

int RobotArmMember::getMaxAngle()
{
	return this->maxAngle;
}

int RobotArmMember::getSafeAngle()
{
	return this->safeAngle;
}

int RobotArmMember::getSensorPin()
{
	return this->sensorPin;
}

int RobotArmMember::getAnalogAngle()
{
		return (int)( this->getAnalogRaw() * sensorLine.m + sensorLine.b );
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

float RobotArmMember::getSensorSlope()
{
	return this->sensorLine.m;
}

float RobotArmMember::getSensorOffset()
{
	return this->sensorLine.b;
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
		DEBUG2("Bad angle input: ", value);
		return;
	}

	int8_t delta = value - this->read();
	uint32_t writeTimeout;
	while (delta != 0)
	{
		this->write(sgn(delta));	// 1 degree at a time
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

void RobotArmMember::printSensorLine()
{
	Serial.print("y = ");
	Serial.print(this->sensorLine.m);
	Serial.print(" * x");
	if (this->sensorLine.b < 0)
		Serial.print(" - ");
	else
		Serial.print(" + ");
	Serial.print(abs(this->sensorLine.b));
	return;
}

void RobotArmMember::sweep()
{
	int initialAngle = this->read();
	this->smooth(minAngle);
	this->smooth(maxAngle);
	this->smooth(initialAngle);
}