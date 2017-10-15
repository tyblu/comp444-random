/*  QuickStatsInt.h - Library for quick descriptive statistics of an array samples[]
 * of size m, assuming a normal distribution.
 *  Based on QuickStats by David Dubins (January 10th, 2016).
 *  Created by Tyler Lucas on October 15th, 2017.
 *  Released into the public domain.
 */

#ifndef QuickStatsInt_h
#define QuickStatsInt_h

#include <Arduino.h>

class QuickStatsInt {
  public:
    QuickStatsInt();
    ~QuickStatsInt();
    int average(int samples[],int m);
//    int g_average(int samples[],int m);
    int minimum(int samples[],int m);
    int maximum(int samples[],int m);
    int stdev(int samples[],int m);
    int stderror(int samples[],int m);
    int CV(int samples[],int m);
    void bubbleSort(int A[],int len);
    int median(int samples[],int m);
    int mode(int samples[],int m,int epsilon);

    int sqrt(int number);
};

#endif
