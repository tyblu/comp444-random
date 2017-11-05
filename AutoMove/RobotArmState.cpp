#include "RobotArmState.h"
#include "C:\Users\tyblu\Documents\repos\comp444-random\AutoMove\RobotArmMember.h"

#define SERVO_PWR_FEEDBACK_ANALOGREAD_POINTS 20	// < uint8_t max and even

RobotArmState::RobotArmState(
	EndEffectorState endEffectorState,
	uint8_t pwrEnablePin,
	uint8_t pwrFeedbackPin,
	RobotArmMember& boom1,
	RobotArmMember& boom2,
	RobotArmMember& turret,
	RobotArmMember& claw
)
	: endEffectorState(endEffectorState)
	, pwrEnablePin(pwrEnablePin)
	, pwrFeedbackPin(pwrFeedbackPin)
	, list()
	, boom1(boom1)
	, boom2(boom2)
	, turret(turret)
	, claw(claw)
	, boom2min()
	, posCenterSonar(), posCenter(), posRest(), posOff()
{
	digitalWrite(pwrEnablePin, LOW);	// ensure servo power is off
	pinMode(pwrEnablePin, OUTPUT);
	pinMode(pwrFeedbackPin, INPUT);

	memberList[0] = &boom1;
	memberList[1] = &boom2;
	memberList[2] = &turret;
	memberList[3] = &claw;
}

// should make this a variadic input to simplify use
// e.g. setBoom2MinMap(90, 83, 100, 68, 110, 55), or ...({90, 83}, {100, 68})
void RobotArmState::setBoom2MinMap(nsTyblu::Pair pairArray[], uint8_t count)
{
	for (uint8_t i = 0; i < count; i++)
		boom2min.put(pairArray[i].key, pairArray[i].val);
}

void RobotArmState::setStoredPosition(NamedPosition posName, int a, int b, int c, int d)
{
	switch (posName)
	{
	case NamedPosition::Center:
		posCenter.set(a, b, c, d);
		break;
	case NamedPosition::CenterSonar:
		posCenterSonar.set(a, b, c, d);
		break;
	case NamedPosition::Rest:
		posRest.set(a, b, c, d);
		break;
	default:
		break;
	}
}

void RobotArmState::goToPos(NamedPosition posName)
{
	boom2.smooth(boom2.getSafeAngle());	// replace with vertical lift function

	switch (posName)
	{
	case NamedPosition::Center:			// don't change claw angle
		boom1.smooth(posCenter.boom1);
		turret.slow(posCenter.turret);
		boom2.smooth(posCenter.boom2);
		break;
	case NamedPosition::CenterSonar:	// don't change claw angle
		boom1.smooth(posCenterSonar.boom1);
		turret.slow(posCenterSonar.turret);
		boom2.smooth(posCenterSonar.boom2);
		break;
	case NamedPosition::Rest:
		claw.write(posRest.claw);
		boom1.smooth(posRest.boom1);
		turret.slow(posRest.turret);
		boom2.smooth(posRest.boom2);
		break;
	default:
		break;
	}
}

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

bool RobotArmState::isServoPowerOn()
{
	int sum = 0;
	for (uint8_t i = 0; i < SERVO_PWR_FEEDBACK_ANALOGREAD_POINTS; i++)
		sum += analogRead(pwrFeedbackPin);
	return (sum > SERVO_PWR_FEEDBACK_ANALOGREAD_POINTS / 2);
}

void RobotArmState::servoPowerOn()
{
	digitalWrite(pwrEnablePin, HIGH);
}

void RobotArmState::servoPowerOff()
{
	digitalWrite(pwrEnablePin, LOW);
}

void RobotArmState::sweep()
{
	int initialAngles[4] = 
		{ boom1.read(), boom2.read(), turret.read(), claw.read() };
	
	for (uint8_t i = 0; i < 4; i++)
		memberList[i]->smooth(memberList[i]->getSafeAngle());

	for (uint8_t i = 0; i < 4; i++)
		memberList[i]->sweep();

	for (uint8_t i = 0; i < 4; i++)
		memberList[i]->smooth(initialAngles[i]);
}

void RobotArmState::attachSafe()
{
	for (uint8_t i = 0; i < 4; i++)
	{
		if (!memberList[i]->attached())
		{
			memberList[i]->write(memberList[i]->getSafeAngle());
			memberList[i]->attach();
		}
		else
			memberList[i]->smooth(memberList[i]->getSafeAngle());
	}
}