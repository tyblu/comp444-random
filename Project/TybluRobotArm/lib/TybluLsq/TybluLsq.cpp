/*
 * TybluLsq.cpp
 *
 *  Created on: Oct 15, 2017
 *      Author: Tyler Lucas <tyblu@live.com>
 */

//#include <Arduino.h>	// only for Serial debug messages
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
//	Serial.println("inside TybluLsq::llsq");
	float top = 0.0, bot = 0.0, xbar = 0.0, ybar = 0.0;

	if ( n == 1 )
	{
		a = 0.0;
		b = y[0];
		return;
	}

	// Average X and Y
	for (int i=0; i<n; i++)
	{
		xbar = xbar + x[i];
		ybar = ybar + y[i];
	}
	xbar = xbar / (float)n;
	ybar = ybar / (float)n;

	// Compute Beta
	for (int i=0; i<n; i++)
	{
		top += ( x[i] - xbar ) * ( y[i] - ybar );
		bot += ( x[i] - xbar ) * ( x[i] - xbar );
	}

	a = top / bot;
	b = ybar - a * xbar;

//	Serial.println("leaving TybluLsq::llsq");
	return;
}

/*
 * As TybluLsq::llsq, but with arrays of ints instead of floats.
 */
void TybluLsq::llsqInt( int n, int x[], int y[], float &a, float &b )
{
	return;
}