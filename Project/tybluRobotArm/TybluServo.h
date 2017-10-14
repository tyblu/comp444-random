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

class TybluServo : public Servo
{
public:
	TybluServo();
	TybluServo(int minAngle, int maxAngle, int sensorPin);

	/**
	 * If value is < 200 its treated as an angle, otherwise as pulse width in
	 * microseconds. If value is < minAngle then it's set to minAngle; if > maxAngle and
	 * < 200 then it's set to maxAngle.
	 */
	void write(int value);	// extends Servo::write(int)

	void setMinAngle(int minAngle);
	void setMaxAngle(int maxAngle);
	void setSensorPin(int sensorPin);

	int getMinAngle();
	int getMaxAngle();
	int getAnalogAngle();

private:
	int minAngle = 0, maxAngle = 180;
	int sensorPin;
	const float sensorSlope, sensorOffset;
	const float analogAngleDeviationLimit;
};

#endif /* TYBLUSERVO_H_ */
