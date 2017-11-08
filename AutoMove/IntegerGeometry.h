/*
* IntegerGeometry.h
*
*  Created on: Oct 31, 2017
*      Author: Tyler Lucas
*
* SOHCAHTOA
*/

#ifndef IntegerGeometry_h
#define IntegerGeomtry_h

#include "inttypes.h"
#include "avr/pgmspace.h"

namespace IntegerGeometry
{
	int sin1000(int angle); // returns 100*sin(angle)
	int cos1000(int angle); // returns 100*cos(angle)

	//int16_t arcSin(int opposite, int hypotenuse);
	//int16_t arcCos(int adjacent, int hypotenuse);

	//int16_t intDiv(int numerator, int divisor); // integer division with rounding to nearest

	// 1000 times sin(angle) from 0 to 90
	const static uint16_t sin1000table[/*91*/] PROGMEM =
	{
		0,
		17,
		35,
		52,
		70,
		87,
		105,
		122,
		139,
		156,
		174,
		191,
		208,
		225,
		242,
		259,
		276,
		292,
		309,
		326,
		342,
		358,
		375,
		391,
		407,
		423,
		438,
		454,
		469,
		485,
		500,
		515,
		530,
		545,
		559,
		574,
		588,
		602,
		616,
		629,
		643,
		656,
		669,
		682,
		695,
		707,
		719,
		731,
		743,
		755,
		766,
		777,
		788,
		799,
		809,
		819,
		829,
		839,
		848,
		857,
		866,
		875,
		883,
		891,
		899,
		906,
		914,
		921,
		927,
		934,
		940,
		946,
		951,
		956,
		961,
		966,
		970,
		974,
		978,
		982,
		985,
		988,
		990,
		993,
		995,
		996,
		998,
		999,
		999,
		1000,
		1000
	};
}

#endif // IntegerGeometry_h