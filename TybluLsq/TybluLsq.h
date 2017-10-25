/*
 * TybluLsq.h
 *
 *  Created on: Oct 15, 2017
 *      Author: Tyler Lucas <tyblu@live.com>
 */

#ifndef TYBLULSQ_H_
#define TYBLULSQ_H_

class TybluLsq
{
public:
	/*
	 * Computes the linear least square fit of Y = a * X + b, minimizing the root-
	 * mean-square error to a set of data points.
	 *
	 * Authors:		John Burkardt		Jul 17, 2011
	 *   adapted by Tyler Lucas			Oct 15, 2017
	 * License: GNU LGPL
	 *
	 * Parameters:
	 * 	n			Number of data points.
	 * 	x[], y[]	Linearly correlated data point arrays, size n.
	 * 	&a, &b		Used for output; the slope and y-intercept of resultant line.
	 */
	static void llsq( int n, float x[], float y[], float &a, float &b );

	/*
	 * As TybluLsq::llsq, but with arrays of ints instead of floats.
	 * 
	 * At the moment it doesn't do anything.
	 */
	static void llsqInt( int n, int x[], int y[], float &a, float &b );

	/*
	 * As TybluLsq::llsq, but reads data sequentially from SD card instead of
	 * potentially large local arrays.
	 * 
	 * At the moment it doesn't do anything.
	 */
	static void llsqSd( /* Not sure what this will need, yet. */);
};

#endif /* TYBLULSQ_H_ */
