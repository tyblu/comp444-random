/*
 * TybluServo.cpp
 *
 *  Created on: Oct 14, 2017
 *      Author: tyblu
 */

#include <inttypes.h>
#include "C:\Users\tyblu\Documents\repos\comp444-random\TybluServo\TybluServo.h"
#include "C:\Users\tyblu\Documents\repos\comp444-random\TybluLsq\TybluLsq.h"
#include "C:\Users\tyblu\Documents\repos\QuickStats\QuickStats.h"

#define TybluServo_DEBUG_MODE
#ifdef TybluServo_DEBUG_MODE
//#	include <Arduino.h>	// only for Serial debug messages
#	define DEBUG1(x) Serial.print("TybluServo : "); Serial.println(x); delay(2)	// note missing ';'
#	define DEBUG2(x,y) Serial.print("TybluServo : "); Serial.print(x); Serial.println(y); delay(2)	// note missing ';'
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
#define MEASUREMENTS_COUNT 40

TybluServo::TybluServo() : Servo()
{
	this->pwmPin = -1;
	this->minAngle = 0;
	this->maxAngle = 180;
	this->safeAngle = (minAngle + maxAngle) / 2;
	this->sensorPin = -1;
	this->sensorSlope = 1.0;
	this->sensorOffset = 0.0;
}

/*
 * Constructor arguments:
 * int pwmPin								Pin used to control servo.
 * int minAngle, int maxAngle 				Allowed range of movement.
 * int safeAngle							Nominal position for servo.
 * int sensorPin 							Angle sensor pin (A0, A1, ...).
 * float sensorSlope, float sensorOffset	Initial angle sensor linear coefficients.
 */
TybluServo::TybluServo(int pwmPin, int min, int max, int safe,
		int sensorPin, float sensorSlope, float sensorOffset) : Servo()
{
	this->pwmPin = pwmPin;
	this->minAngle = min;
	this->maxAngle = max;
	this->safeAngle = safe;
	this->sensorPin = sensorPin;
	this->sensorSlope = sensorSlope;
	this->sensorOffset = sensorOffset;
}

/**
 * Pre-conditions: Servo must be attached, have max/min angles set, and have valid
 * sensor pin.
 * Post-conditions: In addition to preconditions, sensor slope and offset variables
 * are set, allowing for accurate returns from TybluServo::getAnalogAngle.
 *
 * Returns true if it completes successfully, false otherwise. No variables changed.
 *
 * Calibrates servo position sensor by fitting line to measured results while moving
 * between angleA and angleB. Movements starts at angleA, travels to angleB, back to
 * angleA, and it may repeat this several more times. The servo always obeys the
 * minAngle and maxAngle limits, so make sure angleA and angleB are within them or
 * the calibration will not work (returns false).
 */
bool TybluServo::calibrateSensor(int angleA, int angleB)
{
	if (!this->attached())
		return false;

	{	// ensure angleA <= angleB
		int temp = min(angleA, angleB);
		angleB = max(angleA, angleB);
		angleA = temp;
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

		if (iterator == 0)
			continue;

		if (angles[iterator-1] <= angleA)
			direction = 1;
		else if (angles[iterator-1] >= angleB)
			direction = -1;

		angles[iterator] = angles[iterator-1] + direction * CALIB_STEP_SIZE;

	} while (iterator < CALIB_STEPS && total_iterations < CALIB_MAX_ITERATIONS);

	DEBUG1("finished do-while loop");
	DEBUG2("iterator=",iterator);
	DEBUG2("CALIB_STEPS=",CALIB_STEPS);

	if (total_iterations >= CALIB_MAX_ITERATIONS)
		Serial.println("CALIB_MAX_ITERATIONS reached!");

	// least squares fit
	float a, b;

	DEBUG1("Going into TybluLsq::llsq()...");

	// void TybluLsq::llsq( int n, float x[], float y[], float &a, float &b )
	TybluLsq::llsq(CALIB_STEPS, measurements, angles, a, b);

	DEBUG1("Got out of TybluLsq::llsq()");

	sensorSlope = a;
	sensorOffset = b;

	return true;
}

/*
 * Same as calibrateSensor(int angleA, int angleB), using max/min angles.
 */
bool TybluServo::calibrateSensor()
{
	return calibrateSensor(this->minAngle, this->maxAngle);
}

void TybluServo::setMinAngle(int minAngle)
{
	this->minAngle = minAngle;
}

void TybluServo::setMaxAngle(int maxAngle)
{
	this->maxAngle = maxAngle;
}

void TybluServo::setSafeAngle(int safeAngle)
{
	this->safeAngle = safeAngle;
}

void TybluServo::setSensorPin(int sensorPin)
{
	this->sensorPin = sensorPin;
}

void TybluServo::setSensorConstants(float slope, float offset)
{
	this->sensorSlope = slope;
	this->sensorOffset = offset;
}

int TybluServo::getMinAngle()
{
	return this->minAngle;
}

int TybluServo::getMaxAngle()
{
	return this->maxAngle;
}

int TybluServo::getSafeAngle()
{
	return this->safeAngle;
}

int TybluServo::getSensorPin()
{
	return this->sensorPin;
}

int TybluServo::getAnalogAngle()
{
		return (int)( this->getAnalogRaw() * sensorSlope + sensorOffset );
}

int TybluServo::getAnalogRaw()
{
	float measurements[MEASUREMENTS_COUNT];
	float stDev = analogDeviationLimit + 1.0;

	while ( stDev > analogDeviationLimit)
	{
		for (int i=0; i<MEASUREMENTS_COUNT; i++)
			measurements[i] = analogRead(sensorPin);
		qs.bubbleSort(measurements, MEASUREMENTS_COUNT);
		stDev = qs.stdev(measurements, MEASUREMENTS_COUNT);
//		Serial.print(" STDEV=");
//		Serial.println(stDev);
	}

	float mode = qs.mode(measurements, MEASUREMENTS_COUNT, MODE_EPSILON);
//	Serial.print(" MODE=");
//	Serial.print(mode);
//	Serial.print(" SLOPE=");
//	Serial.print( sensorSlope );
//	Serial.print(" OFFSET=");
//	Serial.print( sensorOffset );
//	Serial.write(' ');

	if (mode < MODE_LOWER_LIMIT || mode > MODE_UPPER_LIMIT)
		return 0;
	else
		return (int)mode;
}

float TybluServo::getSensorSlope()
{
	return sensorSlope;
}

float TybluServo::getSensorOffset()
{
	return sensorOffset;
}

/**
 * If value is < 200 its treated as an angle, otherwise as pulse width in
 * microseconds. If value is < minAngle then it's set to minAngle; if > maxAngle and
 * < 200 then it's set to maxAngle.
 */
void TybluServo::write(int value)
{
	if (value < minAngle)
		Servo::write(minAngle);
	else if (value > maxAngle && value < 200)
		Servo::write(maxAngle);
	else							// minAngle > value > maxAngle || value > 200
		Servo::write(value);
}

/*
 * Attempts to pre-set servo position to avoid initial jolts, than attaches.
 * Servo::attach(int) description:
 * 	Attach the given pin to the next free channel, sets pinMode, returns channel
 * 	number or 0 if failure.
 */
uint8_t TybluServo::attach(int pin)
{
	this->pwmPin = pin;
	int currentAnalogAngle = getAnalogAngle();
	if ( currentAnalogAngle >= minAngle && currentAnalogAngle <= maxAngle )
		write(currentAnalogAngle);
	return Servo::attach(pin);
}

/*
 * Preconditions: pwmPin must have been set either in the [full] constructor or
 * TybluServo::attach(int pin) (presumably later followed by a 'detach()).
 *
 * Attaches to previously set pwmPin, calling TybluServo::attach(pwmPin). Returns
 * uint8_t 0 on fail (i.e. bad pwmPin), channel number otherwise.
 */
uint8_t TybluServo::attach()
{
	return this->attach(this->pwmPin);
}

void TybluServo::smooth(int targetAngle)
{
	int currentAngle = this->read();

	if (currentAngle < SMOOTH_ANGLE_MIN || currentAngle > SMOOTH_ANGLE_MAX)
	{
		this->write(targetAngle);		// can't figure out where servo is, just write angle
		return;
	}

	bool isMoving = false;
	int lastCurrentAngle = currentAngle;
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
	}
}