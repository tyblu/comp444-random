/*
 * AutoMoveSD.cpp
 *
 *  Created on: Nov 3, 2017
 *      Author: Tyler Lucas
 */

#include "AutoMoveSD.h"

#define AutoMoveSD_DEBUG_MODE
#ifdef AutoMoveSD_DEBUG_MODE
#	define PRE Serial.print(F("AutoMoveSD : "))
#	define POST delay(2) // note missing ';'
#   define DEBUG0(x) (x); Serial.println(); POST
#   define DEBUG00(x) (x); POST
#	define DEBUG1(x) PRE; Serial.println(x); POST
#   define DEBUG10(x) PRE; Serial.print(x); POST
#	define DEBUG2(x,y) PRE; Serial.print(x); Serial.println(y); POST
#	define DEBUG20(x,y) PRE; Serial.print(x); Serial.print(y); POST
#else
#   define DEBUG0(x) 
#   define DEBUG00(x) 
#	define DEBUG1(x) 
#   define DEBUG10(x) 
#	define DEBUG2(x,y) 
#	define DEBUG20(x,y) 
#endif

 /*
 * Inputs:
 * char * filename			return filename value, 13-byte char array (incl \0)
 		Includes 8 numerical characters for file name, 3 character extension,
 		1 period (extension separator), and null terminator.
 * SdFatEX& arg_sd			FAT volume (etc) object
 * const char * folder		folder to compare for uniqueness
 * const char * prefix		filename prefix, up to 8 char long (not incl \0)
		If less than 8 char the remainder will be filled with unique index,
		the whole point of this function...
 * const char * extension	filename extension, 4-byte char array (incl \0)
 *
 * Post-conditions changes:
 * char * filename			filled out in form [prefix][0-pad][incremented index].[ext]
		0-padding only added if there is room.
		If prefix is longer than 7 char then it is cut to 8 char, no index (or 0-padding).
		If no room for unique index, then it is set to the largest possible (not unique).
		
 * SdFatEX& arg_sd			working directory changed to volume root directory.
 */
void getUniqueFileNameIndex(char * filename, SdFatEX & arg_sd, 
	const char * folder, const char * prefix, const char * extension)
{
	size_t prefixLength = strlen(prefix);
	if (prefixLength > 8)	// no room for index
	{
		strncpy(filename, prefix, 8);
		filename[8] = '\0';	// null terminate so strcat works
		strcat(filename, ".");
		strcat(filename, extension);

		DEBUG2(F("getUniqueFileNameIndex() fin (prefixLength > 8) --> filename = "), filename);

		return;
	}

	uint16_t indexNumber = 0;
	char index[8];
	uint8_t indexLength = (uint8_t)(13 - 4 - prefixLength - 1);

	arg_sd.chdir(true);	// go to root dir
	arg_sd.chdir(folder, true); // go to folder
	do {
		if (indexNumber > pow((uint8_t)10, (uint8_t)(indexLength - 1))) // max index
			return;		// quit, no unique indices left

		DEBUG2(F("indexNumber = "), indexNumber);
		DEBUG2(F("prefixLength= "), prefixLength);
		DEBUG2(F("indexLength = "), indexLength);

		strcpy(filename, prefix);

		DEBUG2(F("strcpy(filename, prefix)         --> filename = "), filename);

		for (uint8_t i = 0; i < indexLength - log10(indexNumber) - 1; i++)
			strcat(filename, "0");

		DEBUG2(F("strcat(filename, \"0\") for loop   --> filename = "), filename);

		itoa(indexNumber, index, 10);
		strcat(filename, index);
		strcat(filename, ".");
		strcat(filename, extension);

		DEBUG2(F("do-while loop finished           --> filename = "), filename);

		indexNumber++;
	} while (arg_sd.exists(filename));
	
	arg_sd.chdir(true);	// back to root dir

	DEBUG2(F("getUniqueFileNameIndex() fin     --> filename = "), filename);

	return;
}

uint16_t pow(uint8_t base, uint8_t exponent)
{
	DEBUG20(F("pow("), base);
	DEBUG00(Serial.write(','));
	DEBUG00(Serial.print(exponent));
	DEBUG00(Serial.print(F(")=")));

	uint16_t result = (uint16_t)base;
	while (--exponent > 0)
	{
		result *= result;
	}

	DEBUG0(Serial.print(result));

	return result;
}

uint8_t log10(uint16_t number)
{
	DEBUG20(F("log10("), number);
	DEBUG00(Serial.print(F(")=")));

	uint8_t result = 0;
	while (number >= 10)
	{
		result++;
		number /= 10;
	}
	DEBUG0(Serial.print(result));

	return result;
}