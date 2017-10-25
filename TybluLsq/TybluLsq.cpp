/*
 * TybluLsq.cpp
 *
 *  Created on: Oct 15, 2017
 *      Author: Tyler Lucas <tyblu@live.com>
 */

#define TybluLsq_DEBUG_MODE
#ifdef TybluLsq_DEBUG_MODE
#	include <Arduino.h>	// only for Serial debug messages
#	define DEBUG1(x) Serial.print("TybluLsq : "); Serial.println(x); delay(2)	// note missing ';'
#	define DEBUG2(x,y) Serial.print("TybluLsq : "); Serial.print(x); Serial.println(y); delay(2)	// note missing ';'
#else
#	define DEBUG1(x)
#	define DEBUG2(x,y)
#endif

#include "TybluLsq.h"

/*
 * Computes the linear least square fit of Y = a * X + b, minimizing the root-
 * mean-square error to a set of data points.
 *
 * Authors:		John Burkardt		Jul 17, 2011
 *   adapted by Tyler Lucas			Oct 15, 2017
 *
 * References:
 * 	https://people.sc.fsu.edu/~jburkardt/cpp_src/llsq/llsq.cpp
 * 	https://people.sc.fsu.edu/~jburkardt/cpp_src/llsq/llsq.html
 *
 * License: GNU LGPL
 *
 * Parameters:
 * 	n			Number of data points.
 * 	x[], y[]	Linearly correlated data point arrays, size n.
 * 	&a, &b		Used for output; the slope and y-intercept of resultant line.
 */
void TybluLsq::llsq( int n, float x[], float y[], float &a, float &b )
{
	float top, bot, xbar, ybar;

	if ( n == 1 )
	{
		a = 0.0;
		b = y[0];
		return;
	}

	DEBUG1(" ");
	DEBUG1("Averaging x and y - printing i, xbar, ybar (sums)");

	// Average X and Y
	xbar = 0.0;
	ybar = 0.0;
	for (int i=0; i<n; i++)
	{
		xbar = xbar + x[i];
		ybar = ybar + y[i];
		DEBUG1(i); DEBUG1(xbar); DEBUG1(ybar); DEBUG1(" ");
	}
	xbar = xbar / (float)n;
	ybar = ybar / (float)n;

	DEBUG2("final xbar=", xbar); DEBUG2("final ybar=", ybar); DEBUG1(" ");

	DEBUG1(" ");
	DEBUG1("Computing Beta - printing i, top, bot, x[i], y[i]");

	// Compute Beta
	top = 0.0;
	bot = 0.0;
	for (int i=0; i<n; i++)
	{
		top = top + ( x[i] - xbar ) * ( y[i] - ybar );
		bot = bot + ( x[i] - xbar ) * ( x[i] - xbar );
		DEBUG1(i); DEBUG1(top); DEBUG1(bot); DEBUG1(x[i]); DEBUG1(y[i]); DEBUG1(" ");
	}

	a = top / bot;
	b = ybar - a * xbar;

	DEBUG2("y=a*x+b, a=", a);
	DEBUG2("y=a*x+b, b=", b);

	return;
}

/*
 * As TybluLsq::llsq, but with arrays of ints instead of floats.
 * 
 * At the moment it doesn't do anything.
 */
void TybluLsq::llsqInt( int n, int x[], int y[], float &a, float &b )
{
	return;		// does nothing.
}

/*
 * As TybluLsq::llsq, but reads data sequentially from SD card instead of
 * potentially large local arrays.
 *
 * At the moment it doesn't do anything.
 */
void TybluLsq::llsqSd( /* Will need something here. */)
{
	return;		// does nothing
}