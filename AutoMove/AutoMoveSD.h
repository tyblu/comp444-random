/*
 * AutoMoveSD.h
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

void getUniqueShortFileName(char * filename, SdFatEX & arg_sd,
	const char * folder, const char * extension);

#endif // AutoMoveSD_h

