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

#define RobotArmMember_DEBUG_MODE
#ifdef RobotArmMember_DEBUG_MODE
#	define DEBUG1(x) Serial.print("RobotArmMember : "); Serial.println(x); delay(2)	// note missing ';'
#	define DEBUG2(x,y) Serial.print("RobotArmMember : "); Serial.print(x); Serial.println(y); delay(2)	// note missing ';'
#else
#	define DEBUG1(x)
#	define DEBUG2(x,y)
#endif

#define SLOW_TIMEOUT_MS 15UL

void RobotArmMember::setLimits(int minAngle, int maxAngle, int safeAngle,
	bool areServoAngles)
{
	if (areServoAngles)
	{
		this->minAngle = servoAngleToTrueAngle(minAngle);
		this->maxAngle = servoAngleToTrueAngle(maxAngle);
		this->safeAngle = servoAngleToTrueAngle(safeAngle);
	}
	else
	{
		this->minAngle = minAngle;
		this->maxAngle = maxAngle;
		this->safeAngle = safeAngle;
	}
}

void RobotArmMember::setAngleConstants(long angleScale1000, long angleOffset)
{
	this->angleScale1000 = angleScale1000;
	this->angleOffset = angleOffset;
}

void RobotArmMember::slow(int angle)
{
	angle = constrain(angle, minAngle, maxAngle);
	int targetServoAngle = trueAngleToServoAngle(angle);
	int currentServoAngle = servo.read();
	int delta = targetServoAngle - currentServoAngle;

	unsigned long writeTimeout;
	while (delta != 0)
	{
		writeTimeout = millis() + SLOW_TIMEOUT_MS;
		while (millis() < writeTimeout) {}	// blocking

		if (delta > 0)
			currentServoAngle++;
		else
			currentServoAngle--;

		this->servo.write(currentServoAngle);
		//this->pos.update();	// should update() here if concurrent code
		delta = targetServoAngle - currentServoAngle;

	}

	this->pos.update();
}

void RobotArmMember::fast(int angle)
{
	angle = constrain(angle, minAngle, maxAngle);
	this->servo.write(trueAngleToServoAngle(angle));
	this->pos.update();
}

int RobotArmMember::getAngle()
{
	return servoAngleToTrueAngle(servo.read());
}

void RobotArmMember::attach()
{
	servo.attach(this->pwmPin);
	this->pos.update();
}

void RobotArmMember::sweep()
{
	int initialAngle = getAngle();
	slow(minAngle);
	slow(maxAngle);
	slow(initialAngle);
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
