/*
 * TybluLsq.cpp
 *
 *  Created on: Oct 15, 2017
 *      Author: Tyler Lucas <tyblu@live.com>
 */

#define TybluLsq_DEBUG_MODE
#ifdef TybluLsq_DEBUG_MODE
#	include <Arduino.h>	// only for Serial debug messages
#	define DEBUG(x) Serial.print("TybluLsq : "); Serial.println(x);
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
	float top = 0.0;
	float bot = 0.0;
	float xbar = 0.0;
	float ybar = 0.0;

	if ( n == 1 )
	{
		a = 0.0;
		b = y[0];
		return;
	}

	DEBUG("right before averaging x and y")

	// Average X and Y
	for (int i=0; i<n; i++)
	{
		xbar = xbar + x[i];
		ybar = ybar + y[i];
	}
	xbar = xbar / (float)n;
	ybar = ybar / (float)n;

	DEBUG("right before computing Beta, printing i, top, bot")

	// Compute Beta
	for (int i=0; i<n; i++)
	{
		top += ( x[i] - xbar ) * ( y[i] - ybar );
		bot += ( x[i] - xbar ) * ( x[i] - xbar );
		DEBUG(i) DEBUG(top) DEBUG(bot) DEBUG(x[i]) DEBUG(y[i])
	}

	a = top / bot;
	b = ybar - a * xbar;

	return;
}

/*
 * As TybluLsq::llsq, but with arrays of ints instead of floats.
 */
void TybluLsq::llsqInt( int n, int x[], int y[], float &a, float &b )
{
	return;
}