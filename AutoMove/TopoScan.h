/*
 * TopoScan.h
 *
 *  Created on: October something, 2017
 *      Author: Tyler Lucas
 */

#ifndef TopoScan_h
#define TopoScan_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "SonarSensor.h"
#include "RobotArmMember.h"
#include "RobotArmState.h"
#include "IntegerGeometry.h"
#include <SdFat.h>

class TopoScan
{
public:
	TopoScan(RobotArmState& state, SonarSensor& sonar);

	void logSonarData(/*SonarSensor & sonar,*/ SdFile & file/*, RobotArmState& state*/);
	void logSonarDataHeader(SdFile & file);
	void logSonarDataEverything(/*SonarSensor & sonar,*/ SdFile & file/*, RobotArmState& state*/);
	void logSonarDataHeaderEverything(SdFile & file);
private:
	RobotArmState& state;
	SonarSensor& sonar;
};

#endif	// TopoScan_h
