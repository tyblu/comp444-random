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

//#define RobotArmMember_DEBUG_MODE	// actually using these #defines for whole document
#ifdef RobotArmMember_DEBUG_MODE
#	define DEBUG1(x) Serial.print("RobotArmMember : "); Serial.println(x); delay(2)	// note missing ';'
#	define DEBUG2(x,y) Serial.print("RobotArmMember : "); Serial.print(x); Serial.println(y); delay(2)	// note missing ';'
#else
#	define DEBUG1(x)
#	define DEBUG2(x,y)
#endif

 /* -------------------------- RobotArmMember -------------------------- */

#define SLOW_TIMEOUT_MS 15UL

RobotArmMember::RobotArmMember(int pwmPin, PositionVector& pos)
	: pwmPin(pwmPin)
	, pos(pos)
	, servo()
{}

void RobotArmMember::setLimits(int minAngle, int maxAngle, int safeAngle)
{
	this->minAngle = minAngle;
	this->maxAngle = maxAngle;
	this->safeAngle = safeAngle;

	DEBUG2(F("minAngle set to "), this->minAngle);
	DEBUG2(F("maxAngle set to "), this->maxAngle);
	DEBUG2(F("safeAngle set to "), this->safeAngle);
}

void RobotArmMember::setAngleConstants(long angleScale1000, long angleOffset)
{
	this->angleScale1000 = angleScale1000;
	this->angleOffset = angleOffset;
}

void RobotArmMember::slow(int angle)
{
	angle = constrain(angle, minAngle, maxAngle);
	int currentServoAngle = servo.read();
	int delta = angle - currentServoAngle;

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
		delta = angle - currentServoAngle;
	}

	this->pos.update(*this);
}

void RobotArmMember::fast(int angle)
{
	angle = constrain(angle, minAngle, maxAngle);
	this->servo.write(angle);
	this->pos.update(*this);
}

void RobotArmMember::safe()
{
	if (!this->servo.attached())
		this->servo.write(safeAngle);
	else
		this->slow(safeAngle);
}

int RobotArmMember::getPhysicalAngle()
{
	return toPhysicalAngle(servo.read());
}

int RobotArmMember::getMinAngle()
{
	return this->minAngle;
}

int RobotArmMember::getMaxAngle()
{
	return this->maxAngle;
}

void RobotArmMember::attach()
{
	servo.attach(this->pwmPin);
	this->pos.update(*this);
}

void RobotArmMember::sweep()
{
	int initialAngle = servo.read();
	slow(minAngle);
	slow(maxAngle);
	slow(initialAngle);
}

int RobotArmMember::toPhysicalAngle(int servoAngle)
{
	long servoAngleL = (long)servoAngle;
	servoAngleL *= this->angleScale1000;
	servoAngleL = (servoAngleL + 500L) / 1000L;	// int division with rounding to nearest
	servoAngleL += this->angleOffset;
	return (int)servoAngleL;
}

int RobotArmMember::toServoAngle(int physicalAngle)
{
	long servoAngleL = (long)physicalAngle;
	servoAngleL -= this->angleOffset;
	servoAngleL = (servoAngleL * 1000L) - 500L;
	servoAngleL = (servoAngleL + abs(this->angleScale1000) / 2) / this->angleScale1000;
	return (int)servoAngleL;
}

PositionVector* RobotArmMember::getPositionVector()
{ 
	return &pos; 
}

Servo* RobotArmMember::getServo()
{ 
	return &servo; 
}

/* -------------------------- end RobotArmMember -------------------------- */

/* -------------------------- PositionVector -------------------------- */

PositionVector::PositionVector(int h, int r, int th) 
	: h(h), r(r), th(th), physicalAngle(0)
{}

void PositionVector::set(int height, int radius, int theta)
{
	this->h = height;
	this->r = radius;
	this->th = th;
}

void PositionVector::update(RobotArmMember & member) {}	// do nothing

int PositionVector::getHeight()
{
	return this->h;
}

int PositionVector::getHeight(int angle) 
{ 
	return this->h; 
}

int PositionVector::getRadius()
{
	return this->r;
}

int PositionVector::getRadius(int angle)
{
	return this->r;
}

int PositionVector::getTheta()
{
	return this->th;
}

bool PositionVector::equals(PositionVector& other)
{
	return (this->h == other.h
		&& this->r == other.r
		&& this->th == other.th);
}

bool PositionVector::equals(int h, int r, int th)
{
	return (this->h == h
		&& this->r == r
		&& this->th == th);
}

void PositionVector::add(int h, int r, int th)
{
	this->h += h;
	this->r += r;
	this->th += th;
}

void PositionVector::add(PositionVector& other)
{
	this->h += other.h;
	this->r += other.r;
	this->th += other.th;
}

int PositionVector::delta(PositionVector &other)
{
	return abs(this->h - other.h) 
		+ abs(this->r - other.r) 
		+ abs(this->th - other.th);
}

PositionVector PositionVector::vectorSum(PositionVector * others[], int count)
{
	PositionVector result = { 0, 0, 0 };
	for (uint8_t i = 0; i < count; i++)
		result.add(*others[i]);
	return result;
}

/* -------------------------- end PositionVector -------------------------- */

/* -------------------------- ClawPositionVector -------------------------- */

ClawPositionVector::ClawPositionVector(long lOffset, long lScale, int incline)
	: PositionVector(0,0,0)
	, lengthOffset(lOffset), lengthScale(lScale)
	, inclineAngle(incline)
{}

void ClawPositionVector::update(RobotArmMember& member)
{
	this->physicalAngle = member.getPhysicalAngle();
	this->r = getRadius(physicalAngle);
	this->h = getHeight(physicalAngle);
}

int ClawPositionVector::getHeight()
{
	return getHeight(this->physicalAngle);
}

int ClawPositionVector::getHeight(int angle)
{
	long heightL = getLength(angle);
	heightL *= IntegerGeometry::sin1000(inclineAngle);
	heightL /= 1000L;
	return (int)heightL;
}

int ClawPositionVector::getRadius()
{
	return getRadius(this->physicalAngle);
}

int ClawPositionVector::getRadius(int angle)
{
	long radiusL = getLength(angle);
	radiusL *= IntegerGeometry::cos1000(inclineAngle);
	radiusL /= 1000L;
	return (int)radiusL;
}

long ClawPositionVector::getLength(int angle)	// private
{
	long lengthL = lengthScale;
	lengthL *= IntegerGeometry::sin1000(angle);
	lengthL /= 1000L;
	lengthL += lengthOffset;

	DEBUG2(F("Claw length = "), lengthL);

	return lengthL;
}

/* ------------------------ end ClawPositionVector ------------------------ */

/* ------------------------ TurretPositionVector ------------------------ */

TurretPositionVector::TurretPositionVector()
	: PositionVector(0,0,0)
{}

void TurretPositionVector::update(RobotArmMember& member)
{ 
	this->th = member.getPhysicalAngle();
}

int TurretPositionVector::getHeight() { return 0; }
int TurretPositionVector::getHeight(int angle) { return 0; }
int TurretPositionVector::getRadius() { return 0; }
int TurretPositionVector::getRadius(int angle) { return 0; }

/* ----------------------- end TurretPositionVector ----------------------- */

/* ----------------------- BoomPositionVector ----------------------- */

BoomPositionVector::BoomPositionVector(long length)
	: PositionVector(0,0,0)
	, length(length)
{}

void BoomPositionVector::update(RobotArmMember& member)
{
	this->physicalAngle = member.getPhysicalAngle();
	this->h = getHeight(physicalAngle);
	this->r = getRadius(physicalAngle);
}

int BoomPositionVector::getHeight()
{
	return getHeight(this->physicalAngle);
}

int BoomPositionVector::getHeight(int angle)
{
	long heightL = this->length;
	heightL *= IntegerGeometry::sin1000(angle);
	heightL /= 1000L;
	return (int)heightL;
}

int BoomPositionVector::getRadius()
{
	return getRadius(this->physicalAngle);
}

int BoomPositionVector::getRadius(int angle)
{
	long radiusL = this->length;
	radiusL *= IntegerGeometry::cos1000(angle);
	radiusL /= 1000L;
	return (int)radiusL;
}

/* ----------------------- end BoomPositionVector ----------------------- */

/* ---------------------- EndEffectorPosition ---------------------- */

EndEffectorPositionVector::EndEffectorPositionVector(
	RobotArmMember & boom1,
	RobotArmMember & boom2,
	RobotArmMember & turret,
	RobotArmMember & claw
)
	: PositionVector(0, 0, 0)
	, boom1(boom1), boom2(boom2), turret(turret), claw(claw)
	, pBoom1(*(boom1.getPositionVector()))
	, pBoom2(*(boom2.getPositionVector()))
	, pTurret(*(turret.getPositionVector()))
	, pClaw(*(claw.getPositionVector()))
{}

void EndEffectorPositionVector::update()
{
	this->h = pBoom1.h + pBoom2.h + pClaw.h;
	this->r = pBoom1.r + pBoom2.r + pClaw.r;
	this->th = pTurret.th;
}

void EndEffectorPositionVector::updateAll()
{
	pBoom1.update(boom1);
	pBoom2.update(boom2);
	pTurret.update(turret);
	pClaw.update(claw);
	this->update();
}

/* ---------------------- end EndEffectorPosition ---------------------- */
