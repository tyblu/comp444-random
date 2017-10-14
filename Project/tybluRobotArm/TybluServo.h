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

	void setMinAngle(int minAngle);
	void setMaxAngle(int maxAngle);
	void setSensorPin(int sensorPin);

	int getMinAngle();
	int getMaxAngle();
	int getAnalogAngle();

private:
	int minAngle = 0, maxAngle = 180;
	int sensorPin;
};

#endif /* TYBLUSERVO_H_ */
