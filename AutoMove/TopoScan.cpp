/*
* TopoScan.cpp
*
*  Created on: October something, 2017
*      Author: Tyler Lucas
*/

#include "TopoScan.h"

TopoScan::TopoScan(RobotArmState& state, SonarSensor& sonar)
	: state(state), sonar(sonar)
{}

void TopoScan::logSonarData(SdFile & file)
{
	//nop
}

void TopoScan::logSonarDataHeader(SdFile & file)
{
	//nop
}

void TopoScan::logSonarDataEverything(SdFile & file)
{
	//nop
}

void TopoScan::logSonarDataHeaderEverything(SdFile & file)
{
	//nop
}