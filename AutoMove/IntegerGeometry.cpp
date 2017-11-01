/*
* IntegerGeometry.cpp
*
*  Created on: Oct 31, 2017
*      Author: Tyler Lucas
*
* SOHCAHTOA
*/

#include "IntegerGeometry.h"

#ifndef abs(x)
#define abs(x) ((x)>0?(x):-(x))
#endif

namespace IntegerGeometry
{
	int16_t bigSin(int angle)
	{
		int16_t result;
		
		if (abs(angle % 180) <= 90)
			result = sin[angle];
		else
			result = sin[180 - angle % 180];

		if (angle < 0)
			return -result;
		else
			return result;
	}

	int16_t bigCos(int angle)
	{
		return bigSin(90 - angle % 180);
	}

	int16_t arcSin(int opposite, int hypotenuse)
	{
		uint16_t div = abs(intDiv(1000 * opposite, hypotenuse));
		int16_t angle = 0;
		int16_t lastAngle = 0;

		while (angle < 90 && sin[angle] < div)
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