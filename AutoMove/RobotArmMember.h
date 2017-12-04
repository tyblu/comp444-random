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

class PositionVector;	// for use in class RobotArmMember

class RobotArmMember
{
public:
	RobotArmMember(int pwmPin, PositionVector& pos);

	void attach();			// Servo::attach(pwmPin) without return

	void slow(int angle);	// Moves servo to target angle at minimum speed.
	void fast(int angle);	// Moves servo to target angle at maximum speed.
	void sweep();			// sweep angle range, current->min->max->current
	void safe();			// go to safeAngle
	void change(int amount);

	void setLimits(int minAngle, int maxAngle, int safeAngle);	// servo angles
	void setAngleConstants(long angleScale1000, long angleOffset);

	PositionVector* getPositionVector();
	Servo* getServo();
	int getPhysicalAngle();
	int getMinAngle();
	int getMaxAngle();

	int toPhysicalAngle(int servoAngle);
	int toServoAngle(int physicalAngle);

protected:
	Servo servo;
	PositionVector& pos;
	long angleOffset, angleScale1000;	// y=ax+b from servo to true angle
	int pwmPin;
	int minAngle, maxAngle, safeAngle;
};

class PositionVector
{
public:
	PositionVector(int h, int r, int th);

	void set(int height, int radius, int theta);

	virtual void update(RobotArmMember& member);

	virtual int getHeight();
	virtual int getHeight(int angle);
	virtual int getRadius();
	virtual int getRadius(int angle);
	virtual int getTheta();

	bool equals(int h, int r, int th);
	bool equals(PositionVector& other);
	void add(int h, int r, int th);
	void add(PositionVector& other);

	int delta(PositionVector& other);

	static PositionVector vectorSum(PositionVector * others[], int count);

	int h, r, th;

protected:
	int physicalAngle;
};

class ClawPositionVector : public PositionVector
{
public:
	ClawPositionVector(long lOffset, long lScale, int incline);

	void update(RobotArmMember& member);

	int getHeight();
	int getHeight(int angle);
	int getRadius();
	int getRadius(int angle);

private:
	long getLength(int angle);

	const int inclineAngle;	// constant angle offset based on configuration
	const long lengthOffset, lengthScale;
};

class TurretPositionVector : public PositionVector
{
public:
	TurretPositionVector();

	void update(RobotArmMember& member);

	int getHeight();
	int getHeight(int angle);
	int getRadius();
	int getRadius(int angle);
};

class BoomPositionVector : public PositionVector
{
public:
	BoomPositionVector(long length);

	void update(RobotArmMember& member);

	int getHeight();
	int getHeight(int angle);
	int getRadius();
	int getRadius(int angle);

private:
	long length;
};

class EndEffectorPositionVector : public PositionVector
{
public:
	EndEffectorPositionVector(
		RobotArmMember& boom1,
		RobotArmMember& boom2,
		RobotArmMember& turret,
		RobotArmMember& claw
	);

	void update();
	void updateAll();

private:
	RobotArmMember & boom1, & boom2, & turret, & claw;
	PositionVector & pBoom1, & pBoom2, & pTurret, & pClaw;
};

#endif /* RobotArmMember_h */