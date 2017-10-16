/*
 * TybluLsq.h
 *
 *  Created on: Oct 15, 2017
 *      Author: tyblu
 */

#ifndef TYBLULSQ_H_
#define TYBLULSQ_H_

class TybluLsq
{
public:
	TybluLsq();

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
	void llsq( int n, float x[], float y[], float &a, float &b );
};

#endif /* TYBLULSQ_H_ */
