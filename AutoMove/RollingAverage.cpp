/*
* RollingAverage.cpp
*
*  Created on: Dec 08, 2017
*      Author: Tyler Lucas <tyblu@live.com>
*/

#include "RollingAverage.h"

RollingAverage::RollingAverage() : val(), avg(0), isAvgUpToDate(true)
{
	for (uint8_t i = 0; i < ROLLING_AVG_DATA_POINTS; i++)
		val[i] = 0;
}
RollingAverage::RollingAverage(uint32_t initialValue) : val(), avg(initialValue), isAvgUpToDate(true)
{
	for (uint8_t i = 0; i < ROLLING_AVG_DATA_POINTS; i++)
		val[i] = initialValue;
}

void RollingAverage::push(uint32_t value)
{
	for (uint8_t i = 1; i < ROLLING_AVG_DATA_POINTS; i++)
		val[i - 1] = val[i];
	val[ROLLING_AVG_DATA_POINTS - 1] = value;
	isAvgUpToDate = false;
}

uint32_t RollingAverage::peek()
{
	return val[ROLLING_AVG_DATA_POINTS - 1];
}

void RollingAverage::set(uint32_t value)
{
	for (uint8_t i = 1; i < ROLLING_AVG_DATA_POINTS; i++)
		val[i] = value;
	avg = value;
	isAvgUpToDate = true;
}

uint32_t RollingAverage::getAverage()
{
	if (!isAvgUpToDate)
		updateAvg();
	return avg;
}

int32_t RollingAverage::getStdDev()
{
	if (!isAvgUpToDate)
		updateAvg();

	int32_t var = 0;
	int32_t dev = 0;
	for (uint8_t i = 0; i < ROLLING_AVG_DATA_POINTS; i++)
	{
		dev = val[i] - avg;
		var += dev * dev;
	}

	var = (var + ROLLING_AVG_DATA_POINTS / 2) / ROLLING_AVG_DATA_POINTS;

	return RollingAverage::sqrt(var);
}

uint8_t RollingAverage::getSize() { return ROLLING_AVG_DATA_POINTS; }

void RollingAverage::updateAvg()
{
	uint32_t sum = 0;
	for (uint8_t i = 0; i < ROLLING_AVG_DATA_POINTS; i++)
		sum += val[i];
	avg = (sum + ROLLING_AVG_DATA_POINTS / 2) / ROLLING_AVG_DATA_POINTS;
	isAvgUpToDate = true;
}

uint32_t RollingAverage::sqrt(uint32_t number)
{
	if (number == 0)
		return 0UL;
	if (number < 3)
		return 1UL;

	uint32_t x = number;
	uint32_t y = 1;
	int32_t error = 1;
	while (x - y > error)
	{
		x = (x + y + 1) / 2;	// half-up rounding with integers (truncation)
		y = (number + x / 2) / x;
	}
	return x;
}