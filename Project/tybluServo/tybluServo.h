/*
 * tybluServo.h - Library for controlling servos on my robotic arm.
 *
 *  Created on: Oct 13, 2017
 *      Author: Tyler Lucas
 */

#include <Arduino.h>
#include <Servo.h>
#include <tybluForceSensor.h>

#ifndef TYBLUSERVO_H
#define TYBLUSERVO_H

class tybluServo
{
public:
	tybluServo(Servo);
	tybluServo(Servo, int, int, tybluForceSensor);
};



#endif /* TYBLUSERVO_H */
