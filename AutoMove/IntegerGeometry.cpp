/*
* IntegerGeometry.cpp
*
*  Created on: Oct 31, 2017
*      Author: Tyler Lucas
*
* SOHCAHTOA
*/

#include "IntegerGeometry.h"
#include <Arduino.h>

//#define IntegerGeometry_DEBUG_MODE
#ifdef IntegerGeometry_DEBUG_MODE
#	define PRE Serial.print(F("IntegerGeometry: "));
#	define POST delay(2) // note missing ';'
#	define DEBUG1(x) PRE; Serial.println(x); POST
#	define DEBUG2(x,y) PRE; Serial.print(x); Serial.println(y); POST
#	define DEBUG3(f,xT,xF) PRE; if(f) { Serial.println(xT); } else { Serial.println(xF); } POST
#else
#	define DEBUG1(x)
#	define DEBUG2(x,y)
#	define DEBUG3(f,xT,xF)
#endif

#ifndef abs(x)
#define abs(x) ((x)>0?(x):-(x))
#endif

namespace IntegerGeometry
{
	int16_t bigSin(int angle)
	{
		int16_t result;

		// map angle to 1st quadrant
		int reducedAngle = ((angle % 360) + 360) % 360;
		if (reducedAngle >= 270)			// quad IV  : sin(x) = -sin(-x)
			return -bigSin(-reducedAngle);
		if (reducedAngle >= 180)			// quad III : sin(x) = -sin(pi+x)
			return -bigSin(180 + reducedAngle);
		if (reducedAngle > 90)				// quad II  : sin(x) = -sin(pi-x)
			return -bigSin(180 - reducedAngle);
		
		DEBUG3((reducedAngle < 0) || (reducedAngle > 90), F("ERROR: reducedAngle="), reducedAngle);

		return pgm_read_word(&IntegerGeometry::sin1000[angle]);
	}

	int16_t bigCos(int angle)
	{
		return bigSin(angle + 90);
	}

	int16_t arcSin(int opposite, int hypotenuse)
	{
		uint16_t div = abs(intDiv(1000 * opposite, hypotenuse));
		int16_t angle = 0;
		int16_t lastAngle = 0;

		while (angle < 90 && pgm_read_word(&IntegerGeometry::sin1000[angle]) < div)
			angle++;

		if (intDiv(1000 * opposite, hypotenuse) < 0)
			return -(angle - 1);
		else
			return angle - 1;
	}

	int16_t arcCos(int adjacent, int hypotenuse)
	{
		return ( 90 - arcSin(adjacent, hypotenuse) ) % 180;
	}

	int16_t intDiv(int numerator, int divisor)
	{
		return (numerator + divisor / 2) / divisor;
	}
}