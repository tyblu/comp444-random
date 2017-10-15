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
#include "QuickStats.h"

class TybluServo : public Servo
{
public:
	TybluServo();
	TybluServo(int minAngle, int maxAngle, int sensorPin,
			float sensorSlope, float sensorOffset, int pt_count);

	/**
	 * If value is < 200 its treated as an angle, otherwise as pulse width in
	 * microseconds. If value is < minAngle then it's set to minAngle; if > maxAngle and
	 * < 200 then it's set to maxAngle.
	 */
	void write(int value);	// extends Servo::write(int)

	void setMinAngle(int minAngle);
	void setMaxAngle(int maxAngle);
	void setSensorPin(int sensorPin);
	void setMeasurementsCount(int pt_count);
	void setSensorConstants(float sensorSlope, float sensorOffset);

	int getMinAngle();
	int getMaxAngle();
	int getAnalogAngle();
	int getMeasurementsCount();

private:
	int minAngle = 0, maxAngle = 180;
	int sensorPin = -1;
	float sensorSlope, sensorOffset;
	const float analogDeviationLimit = 50;
	unsigned int measurementsCount = 100;
	QuickStats qs;
};

#endif /* TYBLUSERVO_H_ */
