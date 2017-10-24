/*
 * TybluServo.h
 *
 *  Created on: Oct 14, 2017
 *      Author: tyblu
 */

#ifndef TYBLUSERVO_H_
#define TYBLUSERVO_H_

#include <Arduino.h>
#include <Servo.h>
#include <inttypes.h>
#include "QuickStats.h"

class TybluServo : public Servo
{
public:
	TybluServo();
	/*
	 * Constructor arguments:
	 * int pwmPin								Pin used to control servo.
	 * int minAngle, int maxAngle 				Allowed range of movement.
	 * int safeAngle							Nominal position for servo.
	 * int sensorPin 							Angle sensor pin (A0, A1, ...).
	 * float sensorSlope, float sensorOffset	Initial angle sensor linear coefficients.
	 */
	TybluServo(int pwmPin, int minAngle, int maxAngle, int safeAngle,
			int sensorPin, float sensorSlope, float sensorOffset);

	/*
	 * If value is < 200 its treated as an angle, otherwise as pulse width in
	 * microseconds. If value is < minAngle then it's set to minAngle; if > maxAngle and
	 * < 200 then it's set to maxAngle.
	 */
	void write(int value);	// extends Servo::write(int)
	uint8_t attach();
	uint8_t attach(int pwmPin);
	void detach();

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
	 * TybluServo::getAnalogAngle.
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

	void setMinAngle(int minAngle);
	void setMaxAngle(int maxAngle);
	void setSafeAngle(int safeAngle);
	void setSensorPin(int sensorPin);
	void setSensorConstants(float sensorSlope, float sensorOffset);

	int getMinAngle();
	int getMaxAngle();
	int getSafeAngle();
	int getSensorPin();
	int getAnalogAngle();
	float getSensorSlope();
	float getSensorOffset();

private:
	int pwmPin;
	int minAngle = 0, maxAngle = 180;
	int safeAngle;
	int sensorPin = -1;
	float sensorSlope, sensorOffset;
	const float analogDeviationLimit = 50;
	unsigned int measurementsCount = 50;
	QuickStats qs;
};

#endif /* TYBLUSERVO_H_ */
