/*
 * tybluServo.h - Library for controlling servos on my robotic arm.
 *
 *  Created on: Oct 13, 2017
 *      Author: Tyler Lucas
 */

#include <Arduino.h>
#include <Servo.h>
#include <TybluServo.h>

#ifndef TYBLUSERVO_H
#define TYBLUSERVO_H

class tybluServo : public Servo
{
public:
	tybluServo();
	tybluServo(int minAngle, int maxAngle, tybluForceSensor fSensor);

	bool setMinAngle(void);
	void setMinAngle(int minAngle);
	bool setMaxAngle(void);
	void setMaxAngle(int maxAngle);
	void setForceSensor(tybluForceSensor fSensor);

	int getMinAngle();
	int getMaxAngle();
	int getAnalogAngle();

private:
	int minAngle = 0, maxAngle = 180;
	tybluForceSensor fSensor;
};



#endif /* TYBLUSERVO_H */
