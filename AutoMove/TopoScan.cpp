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
	file.print(state.getPositionVector()->getHeight());
	file.write(',');
	file.print(state.getPositionVector()->getRadius());
	file.write(',');
	file.print(state.getPositionVector()->getTheta());
	file.write(',');
	file.print(sonar.getMeasurement());
	file.println();
}

void TopoScan::logSonarDataHeader(SdFile & file)
{
	file.print(F("height, radius, theta, sonar measurement"));
	file.println();
}

void TopoScan::logSonarDataEverything(SdFile &file,
	RobotArmMember &member1,
	RobotArmMember &member2,
	RobotArmMember &member3,
	RobotArmMember &member4)
{
	RobotArmMember arr[] = { member1, member2, member3, member4 };

	for (uint8_t i = 0; i < 4; i++)
	{
		file.print(arr[i].getServo()->read());
		file.write(',');
		file.print(arr[i].getPhysicalAngle());
		file.write(',');
		file.print(arr[i].getPositionVector()->getHeight());
		file.write(',');
		file.print(arr[i].getPositionVector()->getRadius());
		file.write(',');
	}
	
	file.print(state.getPositionVector()->getHeight());
	file.write(',');
	file.print(state.getPositionVector()->getRadius());
	file.write(',');
	file.print(state.getPositionVector()->getTheta());
	file.write(',');

	file.print(sonar.getMeasurement());

	file.println();
}

void TopoScan::logSonarDataHeaderEverything(SdFile & file)
{
	file.print(F("boom1 servo angle, boom1 adj angle, boom1 height, boom1 radius,"));
	file.print(F("boom2 servo angle, boom2 adj angle, boom2 height, boom2 radius,"));
	file.print(F("turrent servo angle, turret adj angle, turret height, turret radius,"));
	file.print(F("claw servo angle, claw adj angle, claw height, claw radius,"));
	file.print(F("height, radius, theta, sonar measurement"));
	file.println();
}
