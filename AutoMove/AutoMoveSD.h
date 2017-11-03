/*
 *  .h
 *
 *  Created on: Nov 3, 2017
 *      Author: Tyler Lucas
 */

#ifndef AutoMoveSD_h
#define AutoMoveSD_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "SdFat.h"
#include "inttypes.h"

void getUniqueFileNameIndex(char * filename, SdFatEX & arg_sd,
	const char * folder, const char * prefix, const char * extension);

uint16_t pow(uint8_t base, uint8_t exponent);
uint8_t log10(uint16_t number);
#endif // AutoMoveSD_h

