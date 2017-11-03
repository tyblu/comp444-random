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
	char index[8] = "0000000";
	uint8_t indexLength = (uint8_t)(13 - 4 - prefixLength);

	char newFilename[13];

	arg_sd.chdir(true);	// go to root dir
	arg_sd.chdir(folder, true); // go to folder
	do {
		strcpy(newFilename, prefix);
		
		if (indexNumber > pow((uint8_t)10, indexLength - 1)) // max index
			return;		// quit, no unique indices left

		itoa(indexNumber, index, 10);

		uint16_t pow10 = pow((uint8_t)10, indexLength - 1);
		if (indexNumber < pow10 && pow10 > 0)
		{
			strcat(newFilename, "0");
			pow10 /= 10;
		}

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