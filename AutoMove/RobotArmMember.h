/*
 * RobotArmMember.h
 *
 *  Created on: Oct 14, 2017
 *      Author: Tyler Lucas
 */

#ifndef RobotArmMember_h
#define RobotArmMember_h

#include <Arduino.h>
#include <inttypes.h>
#include <Servo.h>
#include "C:\Users\tyblu\Documents\repos\QuickStats\QuickStats.h"

/* Declarations only, for use in other classes. Remove when unnecessary. */
class RobotArmMember;	// for use in class PositionVector
class Claw;
class Turret;
class Boom;

class PositionVector
{
public:
	PositionVector() : height(0), radius(0), theta(0) { update(); }
	PositionVector(int h, int r, int th) : height(h), radius(r), theta(th) { update(); }

	virtual void update() {}									// override this

	virtual int getHeight() { return getHeight(this->angle); }
	virtual int getHeight(int angle) { return this->height; }
	virtual int getRadius() { return getRadius(this->angle); }
	virtual int getRadius(int angle) { return this->radius; }
	int getTheta() { return this->theta; }

	void add(int h, int r, int th)	// add height, radius, theta to this
	{
		this->height += h;
		this->radius += r;
		this->theta += th;
	}

	void add(PositionVector& other)
	{
		this->height += other.getHeight();
		this->radius += other.getRadius();
		this->theta += other.getTheta();
	}

	static PositionVector vectorSum(PositionVector * others[], int count)
	{
		//int h = 0;
		//int r = 0;
		//int th = 0;
		//for (uint8_t i = 0; i < count; i++)
		//{
		//	h += others[i]->getHeight();
		//	r += others[i]->getRadius();
		//	th += others[i]->getTheta();
		//}
		//return { h, r, th };

		PositionVector result = PositionVector();
		for (uint8_t i = 0; i < count; i++)
			result.add(*others[i]);
		return result;
	}

protected:
	int height, radius, theta, angle;
};

class RobotArmMember
{
public:
	RobotArmMember(int pwmPin, Servo servo)
		: pwmPin(pwmPin)
		, pos()
		, servo()
	{}

	void attach();

	void slow(int value);	// Moves servo to target angle at minimum speed.
	void fast(int value);	// max speed
	void sweep();

	void setLimits(int minAngle, int maxAngle, int safeAngle,
		bool areServoAngles = false);
	void setAngleConstants(long angleScale1000, long angleOffset);

	PositionVector* getPositionVector() { return &pos; }
	Servo* getServo() { return &servo; }
	int getAngle();

protected:
	int servoAngleToTrueAngle(int angle);
	int trueAngleToServoAngle(int angle);

	Servo servo;
	PositionVector pos;
	long angleOffset, angleScale1000;	// y=ax+b from servo to true angle
	int pwmPin;
	int minAngle, maxAngle, safeAngle;
};

class ClawPositionVector : public PositionVector
{
public:
	ClawPositionVector(Claw& clawMember, long lOffset, long lScale, int incline)
		: PositionVector()
		, member(clawMember)
		, lengthOffset(lOffset), lengthScale(lScale)
		, inclineAngle(incline)
	{
		update();
	}

	void update()	// all movement commands should call this
	{
		this->angle = member.getAngle();
		this->radius = getRadius(angle);
		this->height = getHeight(angle);
	}

	int getHeight(int angle)
	{
		long heightL = getLength(angle);
		heightL *= IntegerGeometry::sin1000(inclineAngle);
		heightL /= 1000L;
		return (int)heightL;
	}

	int getRadius(int angle)
	{
		long radiusL = getLength(angle);
		radiusL *= IntegerGeometry::cos1000(inclineAngle);
		radiusL /= 1000L;
		return (int)radiusL;
	}

private:
	long getLength(int angle)
	{
		long lengthL = lengthScale;
		lengthL *= IntegerGeometry::sin1000(angle);
		lengthL /= 1000L;
		lengthL += lengthOffset;
		return lengthL;
	}

	const int inclineAngle;	// constant angle offset based on configuration
	const long lengthOffset, lengthScale;
	Claw& member;
};

class Claw : public RobotArmMember
{
public:
	Claw(long lOffset, long lScale, int pwmPin, Servo servo, int incline)
		: RobotArmMember(pwmPin, servo)
		, pos(*this, lOffset, lScale, incline)
	{
		//
	}

private:
	ClawPositionVector pos;
};

class TurretPositionVector : public PositionVector
{
public:
	TurretPositionVector(Turret& turretMember)
		: PositionVector()
		, member(turretMember)
	{
		update();
	}

	void update() { this->theta = member.getAngle(); }

	int getHeight() { return 0; }
	int getHeight(int angle) { return 0; }
	int getRadius() { return 0; }
	int getRadius(int angle) { return 0; }
	int getTheta() { return this->theta; }

private:
	Turret& member;
};

class Turret : public RobotArmMember
{
public:
	Turret(int pwmPin, Servo servo)
		: RobotArmMember(pwmPin, servo)
		, pos(*this)
	{
		//
	}

private:
	TurretPositionVector pos;
};

class BoomPositionVector : public PositionVector
{
public:
	BoomPositionVector(Boom& boomMember, long length)
		: PositionVector()
		, member(boomMember), length(length)
	{
		update();
	}

	void update()
	{
		this->angle = member.getAngle();
		this->height = getHeight();
		this->radius = getRadius();
	}

	int getHeight() { return getHeight(this->angle); }
	int getHeight(int angle)
	{
		long heightL = this->length;
		heightL *= IntegerGeometry::sin1000(this->angle);
		heightL /= 1000L;
		return (int)heightL;
	}

	int getRadius() { return getRadius(this->angle); }
	int getRadius(int angle)
	{
		long radiusL = this->length;
		radiusL *= IntegerGeometry::cos1000(this->angle);
		radiusL /= 1000L;
		return (int)radiusL;
	}

	int getTheta() { return 0; }

private:
	long length;
	Boom& member;
};

class Boom : public RobotArmMember
{
public:
	Boom(int pwmPin, Servo servo, int length)
		: RobotArmMember(pwmPin, servo)
		, pos(*this, length)
	{
		//
	}

private:
	BoomPositionVector pos;
};
#endif /* RobotArmMember_h */