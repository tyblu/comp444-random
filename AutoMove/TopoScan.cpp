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
	file.print(state.getHeight());
	file.write(',');
	file.print(state.getRadius());
	file.write(',');
	file.print(state.getTheta());
	file.write(',');
	file.print(sonar.getMeasurement());
	file.println();
}

void TopoScan::logSonarDataHeader(SdFile & file)
{
	file.print(F("height, radius, theta, sonar measurement"));
	file.println();
}

void TopoScan::logSonarDataEverything(SdFile & file)
{
	file.print(state.getServo(RobotArmMember::Name::Boom1).read());
	file.write(',');
	file.print(state.getServo(RobotArmMember::Name::Boom1).getAngle());
	file.write(',');
	file.print(state.getServo(RobotArmMember::Name::Boom1).getHeight());
	file.write(',');
	file.print(state.getServo(RobotArmMember::Name::Boom1).getRadius());
	file.write(',');

	file.print(state.getServo(RobotArmMember::Name::Boom2).read());
	file.write(',');
	file.print(state.getServo(RobotArmMember::Name::Boom2).getAngle());
	file.write(',');
	file.print(state.getServo(RobotArmMember::Name::Boom2).getHeight());
	file.write(',');
	file.print(state.getServo(RobotArmMember::Name::Boom2).getRadius());
	file.write(',');

	file.print(state.getServo(RobotArmMember::Name::Turret).read());
	file.write(',');
	file.print(state.getServo(RobotArmMember::Name::Turret).getAngle());
	file.write(',');
	file.print(state.getServo(RobotArmMember::Name::Turret).getHeight());
	file.write(',');
	file.print(state.getServo(RobotArmMember::Name::Turret).getRadius());
	file.write(',');

	file.print(state.getServo(RobotArmMember::Name::Claw).read());
	file.write(',');
	file.print(state.getServo(RobotArmMember::Name::Claw).getAngle());
	file.write(',');
	file.print(state.getServo(RobotArmMember::Name::Claw).getHeight());
	file.write(',');
	file.print(state.getServo(RobotArmMember::Name::Claw).getRadius());
	file.write(',');

	file.print(state.getHeight());
	file.write(',');
	file.print(state.getRadius());
	file.write(',');
	file.print(state.getTheta());
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