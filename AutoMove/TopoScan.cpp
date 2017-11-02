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
	file.print(state.getServo(RobotArmMember::ServoName::Boom1).read());
	file.write(',');
	file.print(state.getServo(RobotArmMember::ServoName::Boom1).getAngle());
	file.write(',');
	file.print(state.getServo(RobotArmMember::ServoName::Boom1).getHeight());
	file.write(',');
	file.print(state.getServo(RobotArmMember::ServoName::Boom1).getRadius());
	file.write(',');

	file.print(state.getServo(RobotArmMember::ServoName::Boom2).read());
	file.write(',');
	file.print(state.getServo(RobotArmMember::ServoName::Boom2).getAngle());
	file.write(',');
	file.print(state.getServo(RobotArmMember::ServoName::Boom2).getHeight());
	file.write(',');
	file.print(state.getServo(RobotArmMember::ServoName::Boom2).getRadius());
	file.write(',');

	file.print(state.getServo(RobotArmMember::ServoName::Turret).read());
	file.write(',');
	file.print(state.getServo(RobotArmMember::ServoName::Turret).getAngle());
	file.write(',');
	file.print(state.getServo(RobotArmMember::ServoName::Turret).getHeight());
	file.write(',');
	file.print(state.getServo(RobotArmMember::ServoName::Turret).getRadius());
	file.write(',');

	file.print(state.getServo(RobotArmMember::ServoName::Claw).read());
	file.write(',');
	file.print(state.getServo(RobotArmMember::ServoName::Claw).getAngle());
	file.write(',');
	file.print(state.getServo(RobotArmMember::ServoName::Claw).getHeight());
	file.write(',');
	file.print(state.getServo(RobotArmMember::ServoName::Claw).getRadius());
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