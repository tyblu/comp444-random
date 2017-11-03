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
void getUniqueShortFileName(char * filename, SdFatEX & arg_sd, const char * folder, const char * extension)
{
	//	String newFilename = String(ULONG_MAX, DEC);
	String str = "01234567";
	String newFilename = str + ".ext";
	uint8_t fileNumber = 0;

	arg_sd.chdir(true);	// go to root dir
	arg_sd.chdir(folder, true); // go to folder
	do {
		if (fileNumber > 999)
			return;		// quit, too many files
		else if (fileNumber > 100)
			newFilename = "sonar";
		else if (fileNumber > 10)
			newFilename = "sonar0";
		else
			newFilename = "sonar00";

		newFilename.concat(fileNumber++);
		newFilename.concat('.');
		newFilename.concat(extension);
		newFilename.toCharArray(filename, 13);
	} while (arg_sd.exists(filename));
	arg_sd.chdir(true);	// back to root dir
	return;
}