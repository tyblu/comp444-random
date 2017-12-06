/*
 * AutoMoveSD.cpp
 *
 *  Created on: Nov 3, 2017
 *      Author: Tyler Lucas
 */

#include "AutoMoveSD.h"

#define AutoMoveSD_DEBUG_MODE
#define getUniqueIncrementedFileName_DEBUG_MODE
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
#endif // AutoMoveSD_DEBUG_MODE
#ifdef getUniqueIncrementedFileName_DEBUG_MODE
#   define DEBUG0_UNIQUE(x) DEBUG0(x)
#   define DEBUG00_UNIQUE(x) DEBUG00(x)
#	define DEBUG1_UNIQUE(x) DEBUG1(x)
#   define DEBUG10_UNIQUE(x) DEBUG10(x)
#	define DEBUG2_UNIQUE(x,y) DEBUG2(x,y)
#	define DEBUG20_UNIQUE(x,y) DEBUG20(x,y)
#else
#   define DEBUG0_UNIQUE(x) 
#   define DEBUG00_UNIQUE(x) 
#	define DEBUG1_UNIQUE(x) 
#   define DEBUG10_UNIQUE(x) 
#	define DEBUG2_UNIQUE(x,y) 
#	define DEBUG20_UNIQUE(x,y) 
#endif // getUniqueIncrementedFileName_DEBUG_MODE

#define SPI_DIVIDER_MAX 32+1
#define MAX_PATH_NAME_SIZE 9

namespace AutoMoveSD
{
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
	void getUniqueIncrementedFileName(char * filename, SdFatEX & arg_sd,
		const char * folder, const char * prefix, const char * extension)
	{
		size_t prefixLength = strlen(prefix);
		if (prefixLength > 8)	// no room for index
		{
			strncpy(filename, prefix, 8);
			filename[8] = '\0';	// null terminate so strcat works
			strcat(filename, ".");
			strcat(filename, extension);

			DEBUG2_UNIQUE(F("getUniqueFileNameIndex() fin (prefixLength > 8) --> filename = "), filename);

			return;
		}

		uint16_t indexNumber = 0;
		char index[8];
		uint8_t indexLength = (uint8_t)(13 - 4 - prefixLength - 1);

		if (!arg_sd.chdir(true))	// go to root dir
		{
			DEBUG1_UNIQUE(F("getUnique...() : sd.chdir() to root failed."));
			strcpy(filename, "err.txt");
			return;
		}

		if (!arg_sd.chdir(folder, true)) // go to folder
		{
			DEBUG1_UNIQUE(F("getUnique...() : sd.chdir() to root failed."));
			strcpy(filename, "err.txt");
			return;
		}

		do {
			if (indexNumber > AutoMoveSD::pow((uint8_t)10, (uint8_t)(indexLength - 1))) // max index
				return;		// quit, no unique indices left

			uint8_t numberLength = AutoMoveSD::log10(indexNumber) + 1;

			DEBUG2_UNIQUE(F("indexNumber = "), indexNumber);
			DEBUG2_UNIQUE(F("prefixLength= "), prefixLength);
			DEBUG2_UNIQUE(F("indexLength = "), indexLength);
			DEBUG2_UNIQUE(F("numberLength= "), numberLength);

			strcpy(filename, prefix);

			DEBUG2_UNIQUE(F("strcpy(filename, prefix)         --> filename = "), filename);

			for (uint8_t i = 0; i < indexLength - numberLength; i++)
				strcat(filename, "0");

			DEBUG2_UNIQUE(F("strcat(filename, \"0\") for loop   --> filename = "), filename);

			itoa(indexNumber, index, 10);
			strcat(filename, index);
			strcat(filename, ".");
			strcat(filename, extension);

			DEBUG2_UNIQUE(F("do-while loop finished           --> filename = "), filename);

			indexNumber++;
		} while (arg_sd.exists(filename));

		arg_sd.chdir(true);	// back to root dir

		DEBUG2_UNIQUE(F("getUniqueFileNameIndex() fin     --> filename = "), filename);

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
			result *= base;
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

	bool startScript(SdFatEX & arg_sd, int csPin, int spiClockDivider)
	{
		/* Initialize at the highest speed supported by the board that is
		 * not over 50 MHz. Try a lower speed if SPI errors occur. */
		bool beginSucceeded = arg_sd.begin(csPin, SD_SCK_MHZ(F_CPU / spiClockDivider));

		if (!beginSucceeded)
		{
			DEBUG2(F("startScript() failed at F_CPU/"), spiClockDivider);

			if (spiClockDivider < SPI_DIVIDER_MAX)
			{
				spiClockDivider *= 2;
				return startScript(arg_sd, csPin, spiClockDivider);
			}

			arg_sd.initErrorPrint();
			return false;	// failed all clock speeds
		}

		return beginSucceeded;
	}

	bool openNewFile(SdFatEX & sd, const char * dirName, SdFile & f, const char * fname)
	{
		if (strlen(dirName) > MAX_PATH_NAME_SIZE)
		{
			DEBUG20(F("openNewFile() : dir name too long, strlen("), dirName);
			DEBUG00(Serial.print(F(")=")));
			DEBUG00(Serial.print(strlen(dirName)));
			DEBUG00(Serial.print(F(" > ")));
			DEBUG00(Serial.println(MAX_PATH_NAME_SIZE));
			return false;
		}

		char path[MAX_PATH_NAME_SIZE];
		char pathFwdSlash[MAX_PATH_NAME_SIZE];

		if (dirName[0] == '/')				// remove prepended slash
		{
			strncpy(path, &dirName[1], MAX_PATH_NAME_SIZE);	// dirty C-string trick, copies dir[1] onwards
			strncpy(pathFwdSlash, dirName, MAX_PATH_NAME_SIZE);
		}
		else
		{
			strncpy(path, dirName, MAX_PATH_NAME_SIZE);
			strncpy(path, "/", MAX_PATH_NAME_SIZE);
			strcat(path, dirName);
		}

		if (!sd.chdir());						// goto root dir
		{
			DEBUG1(F("sd.chdir() to root failed."));
			return false;
		}

		if (!sd.exists(path))
		{
			if (!sd.mkdir(path))
			{
				DEBUG2(F("sd.mkdir(path) failed, path = "), path);
				return false;
			}
		}

		if (!sd.chdir(pathFwdSlash, true))
		{
			DEBUG2(F("openNewFile() : sd.chdir(pathFwdSlash) failed, path = "), pathFwdSlash);
			return false;
		}

		return f.open(fname, O_CREAT | O_WRITE);
	}

	bool initTopo(SdFatEX & sd, const char * dirName, const char * pre, SdFile & f)
	{
		char filename[13];
		AutoMoveSD::getUniqueIncrementedFileName(filename, sd, dirName, pre, "txt");
		return AutoMoveSD::openNewFile(sd, dirName, f, filename);
	}

	void serialPrintFileNames(SdFile * fileArr, int fileCount)
	{
		char fname[13];
		for (uint8_t i = 0; i < fileCount; i++)
		{
			if (i > 0) { Serial.print(F(", ")); }
			fileArr[i].getName(fname, 13);
			Serial.print(fname);
		}
	}

	void serialPrintlnFileNames(SdFile * fileArr, int fileCount)
	{
		serialPrintFileNames(fileArr, fileCount);
		Serial.println();
	}
}