/*
 * AutoMoveSD.cpp
 *
 *  Created on: Nov 3, 2017
 *      Author: Tyler Lucas
 */

#include "AutoMoveSD.h"

 /*
 * Input:
 * char* filename			return filename value, 13-byte char array (incl \0)
 *		Includes 8 numerical characters for file name, 3 character extension,
 *		and 2 characters for period (extension separator) and null terminator.
 *
 * SdFatEX& arg_sd			FAT volume (etc) object
 * const char * folder		folder to compare for uniqueness
 * const char * extension	filename extension, 4-byte char array (incl \0)
 *
 * Post-conditions: char * filename changed to unique filename.
 *	arg_sd working directory changed to volume root directory.
 */
void getUniqueFileNameIndex(char * filename, SdFatEX & arg_sd, 
	const char * folder, const char * prefix, const char * extension)
{
	size_t prefixLength = strlen(prefix);
	if (prefixLength > 8)	// no room for index
		return;

	uint16_t indexNumber = 0;
	char index[8];
	char indexPadding[8];
	uint8_t indexLength = (uint8_t)(13 - 4 - prefixLength - 1);
	const static char padding[] PROGMEM = "0000000000";

	char newFilename[13];

	arg_sd.chdir(true);	// go to root dir
	arg_sd.chdir(folder, true); // go to folder
	do {
		strcpy(newFilename, prefix);
		newFilename[prefixLength] = '\0';	// ensure NULL termination
		
		if (indexNumber > pow((uint8_t)10, indexLength - 1)) // max index
			return;		// quit, no unique indices left

		strncpy_P(indexPadding, padding, indexLength - log10(indexNumber) - 2);
		indexPadding[indexLength - log10(indexNumber) - 1] = '\0';
		strcat(newFilename, indexPadding);
		itoa(indexNumber, index, 10);
		strcat(newFilename, index);
		strcat(newFilename, ".");
		strcat(newFilename, extension);

		indexNumber++;
	} while (arg_sd.exists(newFilename));

	strcpy(filename, newFilename);
	
	arg_sd.chdir(true);	// back to root dir

	return;
}

uint16_t pow(uint8_t base, uint8_t exponent)
{
	uint16_t result = (uint16_t)base;
	while (--exponent > 0)
	{
		result *= result;
	}
	return result;
}

uint8_t log10(uint16_t number)
{
	uint8_t result = 0;
	while (number >= 10)
	{
		result++;
		number /= 10;
	}
	return result;
}