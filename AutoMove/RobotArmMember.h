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

const static char * const servoStr[] PROGMEM = 
	{ "Boom1", "Boom2", "Turret", "Claw" , "ERROR" };

class RobotArmMember;	// declaration only, for use in class PositionVector
class PositionVector
{
public:
	PositionVector(int length, int angle, int theta);

	void update(RobotArmMember& ram);

	int getHeight();
	int getHeight(int angle);
	int getRadius();
	int getRadius(int angle);
	int getTheta();

private:
	void updateHeight();
	void updateRadius();
	int height, radius, theta, angle, length;
};

class RobotArmMember : public Servo
{
public:
	enum Name { Boom1, Boom2, Turret, Claw };

	/*
	 * Constructor arguments:
	 * Name name							Boom1, Boom2, Turret, or Claw.
	 * int pwmPin								Pin used to control servo.
	 * int minAngle, int maxAngle 				Allowed range of movement.
	 * int safeAngle							Nominal position for servo.
	 * int sensorPin 							Angle sensor pin (A0, A1, ...).
	 * float sensorSlope, float sensorOffset	Initial angle sensor linear coefficients.
	 */
	RobotArmMember(Name name, uint16_t length, int pwmPin, 
			int minAngle, int maxAngle, int safeAngle,
			int sensorPin);

	/*
	 * If value is < 200 its treated as an angle, otherwise as pulse width in
	 * microseconds. If value is < minAngle then it's set to minAngle; if > maxAngle and
	 * < 200 then it's set to maxAngle.
	 */
	void write(int value);	// extends Servo::write(int)
	uint8_t attach();

	/*
	 * Preconditions: [optional] Servo is attached and at its currently set angle.
	 * (Otherwise the servo will jump.) Suggest using calibrated analog angle sensor.
	 *
	 * Limits acceleration and speed while moving from current angle to target.
	 */
	void smooth(int value);

	// Moves servo to target angle at minimum speed.
	void slow(int value);

	/*
	 * Pre-conditions: Servo must be attached, have max/min angles set, and have
	 * valid sensor pin.
	 * Post-conditions (if successful/returns true): In addition to preconditions,
	 * sensor slope and offset variables are set, allowing for accurate returns from
	 * RobotArmMember::getAnalogAngle.
	 *
	 * Returns true if it completes successfully, false otherwise.
	 *
	 * Calibrates servo position sensor by fitting line to measured results while
	 * moving between angleA and angleB. Movements starts at angleA, travels to
	 * angleB, back to angleA, and it may repeat this several more times. The servo
	 * always obeys the minAngle and maxAngle limits, so make sure angleA and angleB
	 * are within them or the calibration will not work (returns false).
	 */
	bool calibrateSensor(int angleA, int angleB);
	/*
	 * Same as calibrateSensor(int angleA, int angleB), using max/min angles.
	 */
	bool calibrateSensor();

	void sweep();

	void setName(Name name);
	void setMinServoAngle(int minAngle);
	void setMaxServoAngle(int maxAngle);
	void setSafeServoAngle(int safeAngle);
	void setSensorPin(int sensorPin);
	void setSensorConstants(long sensorScale1000, long sensorOffset);
	void setAngleConstants(long angleScale1000, long angleOffset);

	Name getName();
	void getNameStr(char nameStr[7]);	// returns name in 7 char array
	int getAngle();		// returns angle from Servo::read() with offset
	int getHeight();	// height of axis at end from axis at start [mm]
	int getHeight(int angle);	// height if at this angle
	int getRadius();	// radius of axis at end from axis at start [mm]
	int getRadius(int angle);	// radius if at this angle
	int getMinAngle();
	int getMinServoAngle();
	int getMaxAngle();
	int getMaxServoAngle();
	int getSafeServoAngle();
	int getMaxHeight();
	int getMinHeight();
	int getMaxRadius();
	int getMinRadius();
	int getSensorPin();
	int getAnalogAngle();
	uint16_t getLength();

private:
	int getAnalogRaw();
	int servoAngleToTrueAngle(int angle);
	int trueAngleToServoAngle(int angle);

	Name name;		// Boom1, Boom2, Turret, or Claw
	const uint16_t length;	// from axis to axis
	long angleOffset, angleScale1000;	// y=ax+b from servo to true angle
	long sensorOffset, sensorScale1000;	// y=ax+b from sensor to servo angle
	int pwmPin = -1;
	int minAngle = 0, maxAngle = 180;	// max and min servo angles
	int safeAngle = 90;					// safe servo angle in all states
	int sensorPin = -1;
	const float analogDeviationLimit = 50;
	unsigned int measurementsCount = 50;
	QuickStats qs;

	int maxHeight, minHeight, maxRadius, minRadius;
};

#endif /* RobotArmMember_h */