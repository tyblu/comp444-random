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

class RobotArmMember : public Servo
{
public:
	enum ServoName { Boom1, Boom2, Turret, Claw };

	/* Line equation: y = m * x + b */
	class Line
	{
	public:
		Line(float m, float b) : m(m), b(b) {}
		float valueAt(float x) { return m*x + b; }
		float m, b;
	};

	/*
	 * Constructor arguments:
	 * ServoName name							Boom1, Boom2, Turret, or Claw.
	 * int angleOffset							Offset used for state calculations.
	 * int pwmPin								Pin used to control servo.
	 * int minAngle, int maxAngle 				Allowed range of movement.
	 * int safeAngle							Nominal position for servo.
	 * int sensorPin 							Angle sensor pin (A0, A1, ...).
	 * float sensorSlope, float sensorOffset	Initial angle sensor linear coefficients.
	 */
	RobotArmMember(ServoName name, uint16_t length, int angleOffset, int pwmPin, 
			int minAngle, int maxAngle, int safeAngle,
			int sensorPin, float sensorSlope, float sensorOffset);

	/*
	 * If value is < 200 its treated as an angle, otherwise as pulse width in
	 * microseconds. If value is < minAngle then it's set to minAngle; if > maxAngle and
	 * < 200 then it's set to maxAngle.
	 */
	void write(int value);	// extends Servo::write(int)
	uint8_t attach();
//	uint8_t attach(int pwmPin);

	/*
	 * Preconditions: [optional] Servo is attached and at its currently set angle.
	 * (Otherwise the servo will jump.) Suggest using calibrated analog angle sensor.
	 *
	 * Limits acceleration and speed while moving from current angle to target.
	 */
	void smooth(int value);

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


	/* Prints y = m * x + b string. */
	void printSensorLine();

	void setName(ServoName name);
	void setAngleOffset(int angleOffset);
	void setMinAngle(int minAngle);
	void setMaxAngle(int maxAngle);
	void setSafeAngle(int safeAngle);
	void setSensorPin(int sensorPin);
	void setSensorConstants(float sensorSlope, float sensorOffset);

	ServoName getName();
	int getAngleOffset();
	int getAngle();		// returns angle from Servo::read() with offset
	int getHeight();	// height of axis at end from axis at start [mm]
	int getRadius();	// radius of axis at end from axis at start [mm]
	int getMinAngle();
	int getMaxAngle();
	int getSafeAngle();
	int getSensorPin();
	int getAnalogAngle();
	float getSensorSlope();
	float getSensorOffset();

private:
	int getAnalogRaw();

	Line sensorLine;
	ServoName name;		// Boom1, Boom2, Turret, or Claw
	const uint16_t length;	// from axis to axis
	int angleOffset;	// servo angle to member angle from horizontal
	int pwmPin = -1;
	int minAngle = 0, maxAngle = 180;	// max and min servo angles
	int safeAngle = 90;					// safe servo angle in all states
	int sensorPin = -1;
	const float analogDeviationLimit = 50;
	unsigned int measurementsCount = 50;
	QuickStats qs;
};

#endif /* RobotArmMember_h */