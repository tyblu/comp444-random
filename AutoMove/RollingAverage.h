/*
* RollingAverage.h
*
*  Created on: Dec 08, 2017
*      Author: Tyler Lucas <tyblu@live.com>
*/

#ifndef RollingAverage_h
#define RollingAverage_h

#include "inttypes.h"

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#define ROLLING_AVG_DATA_POINTS 3

class RollingAverage
{
public:
	RollingAverage();
	RollingAverage(uint32_t initialValue);

	void push(uint32_t value);
	void set(uint32_t value);
	uint32_t peek();

	uint32_t getAverage();
	uint8_t getSize();
	int32_t getStdDev();

private:
	void updateAvg();

	uint32_t val[ROLLING_AVG_DATA_POINTS];
	uint32_t avg;
	bool isAvgUpToDate;

	static uint32_t sqrt(uint32_t number);
};

#endif	// RollingAverage_h
