/* QuickStatsInt.cpp - Library for quick descriptive statistics of an array
 * samples[] of size m.
 *  Based on QuickStats by David Dubins (January 10th, 2016) - http://pb435.pbworks/com
 *  Created by Tyler Lucas on October 15th, 2017.
 *  Released into the public domain.
 *  Requires Arduino 1.6.6 or greater.
 */

#include "Arduino.h"
#include "QuickStatsInt.h"
#include <math.h>

QuickStatsInt::QuickStatsInt(){/*nothing to construct*/}
QuickStatsInt::~QuickStatsInt(){/*nothing to destruct*/}

int QuickStatsInt::average(int samples[], int m)
{
	long sum = 0;
		for (int i=0; i<m; i++)
	sum += samples[i];
	return sum / m;
}

/*
 * This needs integer implementations of log and exp (no float intermediaries).
 * Might find C algorithm of log(int) at
 * https://en.wikipedia.org/wiki/Baby-step_giant-step.
 */
//int QuickStatsInt::g_average(int samples[], int m)
//{
//	long sum = 0;
//		for(int i=0; i<m; i++)
//	sum += log( samples[i] );
//	return exp( sum / m );
//}

int QuickStatsInt::minimum(int samples[], int m)
{
	int sorted[m];   //Define and initialize sorted array
	for(int i=0; i<m; i++)
		sorted[i] = samples[i];
	bubbleSort( sorted,m );  // Sort the values
	return( sorted[0] ); // first element is the minimum
}

int QuickStatsInt::maximum(int samples[], int m)
{
	int sorted[m];   //Define and initialize sorted array
		for(int i=0;i<m;i++)
	sorted[i] = samples[i];
	bubbleSort( sorted,m );  // Sort the values
	return( sorted[m-1] );   // last element is the maximum
}

int QuickStatsInt::stdev(int samples[], int m)
{
	int avg = 0;
	int sum = 0;
	avg = average( samples,m );
	for(int i=0; i<m; i++)
		sum += (samples[i] - avg) * (samples[i] - avg);
	return (int)QuickStatsInt::sqrt( sum / (m-1) );
}

int QuickStatsInt::stderror(int samples[], int m)
{
	int temp1 = 0;
	temp1 = stdev( samples, m );
	return temp1 / sqrt(m);
}

//Coefficient of variation (%RSD, or relative stdev)
int QuickStatsInt::CV(int samples[], int m)
{
	int avg = 0;
	int sd = 0;
	avg = average( samples, m );
	sd = stdev( samples, m );
	return 100 * sd / avg;
}

void QuickStatsInt::bubbleSort( int A[], int len)
{
	unsigned long newn;
	unsigned long n=len;
	int temp = 0;
	do {
	newn = 1;
	for(int p=1; p<len; p++)
	{
		if( A[p-1] > A[p] )
		{
			temp = A[p];           //swap places in array
			A[p] = A[p-1];
			A[p-1] = temp;
			newn = p;
		}
	}
	n = newn;
	} while( n>1 );
}

int QuickStatsInt::median(int samples[], int m) //calculate the median
{
	//First bubble sort the values: https://en.wikipedia.org/wiki/Bubble_sort
	int sorted[m];   //Define and initialize sorted array.
	int temp = 0;      //Temporary float for swapping elements
	/*Serial.println("Before:");
	for(int j=0;j<m;j++){
		Serial.println(samples[j]);
	}*/
	for ( int i=0; i<m; i++ )
		sorted[i]=samples[i];
	bubbleSort( sorted, m );  // Sort the values
	/*Serial.println("After:");
	for(int i=0;i<m;i++){
		Serial.println(sorted[i]);
	}*/
	if ( bitRead(m,0) == 1 )  //If the last bit of a number is 1, it's odd. This is equivalent to "TRUE". Also use if m%2!=0.
		return sorted[m/2]; //If the number of data points is odd, return middle number.
	else
		return (sorted[(m/2)-1]+sorted[m/2])/2; //If the number of data points is even, return avg of the middle two numbers.
}

int QuickStatsInt::mode(int samples[], int m, int epsilon) //calculate the mode.
//epsilon is the tolerance for two measurements to be equivalent.
{
	//First bubble sort the values: https://en.wikipedia.org/wiki/Bubble_sort
	int sorted[m];   //Temporary array to sort values.
	int temp = 0;      //Temporary float for swapping elements
	int unique[m];   //Temporary array to store unique values
	int uniquect[m]; //Temporary array to store unique counts
	/*Serial.println("Before:");
	for(int i=0;i<m;i++){
		Serial.println(samples[i]);
	}*/
	for( int i=0; i<m; i++ )
		sorted[i] = samples[i];
	bubbleSort( sorted, m );  // Sort the values
	/*Serial.println("Sorted:");
	for(int i=0;i<m;i++){
		Serial.println(sorted[i]);
	}*/
	// Now count the number of times each unique number appears in the sorted array.
	unique[0] = sorted[0];
	uniquect[0] = 1;
	int p = 0; // counter for # unique numbers
	int maxp = 0;
	int maxidx = 0;
	for( int i=1; i<m; i++)
	{
		if( abs(sorted[i]-sorted[i-1]) < epsilon )
		{
			uniquect[p]++;  //if same number again, add to count
			if( uniquect[p] > maxp )
			{
				maxp = uniquect[p];
				maxidx = p;
			}
		} else
		{
			p++;
			unique[p] = sorted[i];
			uniquect[p] = 1;
		}
	}
	/*for(int i=0;i<p+1;i++){
	Serial.println("Num: " + (String)unique[i] +"   Count: " + (String)uniquect[i]);
	}*/
	if ( maxp > 1 )
		return unique[maxidx]; //If there is more than one mode, return the lowest one.
	else
		return 0; //If there is no mode, return a zero.
}

// https://en.wikipedia.org/wiki/Integer_square_root
int QuickStatsInt::sqrt(int number)
{
	if (number < 0)
		return -1;

	int shift = 2;
	int numberShifted = number >> shift;
	while ( numberShifted != 0 && numberShifted != number )
	{
		shift += 2;
		numberShifted = number >> shift;
	}
	shift -= 2;

	int result = 0;
	while ( shift >= 0 )
	{
		result = result << 1;
		int candidateResult = result + 1;
		if ( (candidateResult * candidateResult) <= (number >> shift) )
			result = candidateResult;
		shift -= 2;
	}

	return result;
}

