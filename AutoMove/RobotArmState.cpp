#include "RobotArmState.h"
#include "C:\Users\tyblu\Documents\repos\comp444-random\AutoMove\RobotArmMember.h"

RobotArmState::RobotArmState(EndEffectorState endEffectorState,
	RobotArmMember& boom1,
	RobotArmMember& boom2,
	RobotArmMember& turret,
	RobotArmMember& claw
)
	: endEffectorState(endEffectorState)
	, list()
	, boom1(boom1)
	, boom2(boom2)
	, turret(turret)
	, claw(claw)
{}

RobotArmMember& RobotArmState::getServo(RobotArmMember::ServoName name)
{
	switch (name)
	{
	case RobotArmMember::ServoName::Boom1: return boom1;
	case RobotArmMember::ServoName::Boom2: return boom2;
	case RobotArmMember::ServoName::Turret: return turret;
	case RobotArmMember::ServoName::Claw: return claw;
	default:
		return;
	}
}

uint16_t RobotArmState::getRadius()
{
	return boom1.getRadius() + boom2.getRadius() + claw.getRadius();
}

uint16_t RobotArmState::getHeight()
{
	return boom1.getHeight() + boom2.getHeight() + claw.getHeight();
}

uint16_t RobotArmState::getTheta()
{
	return turret.getAngle();
}