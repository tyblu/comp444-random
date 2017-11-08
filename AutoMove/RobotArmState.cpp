#include "RobotArmState.h"
#include "C:\Users\tyblu\Documents\repos\comp444-random\AutoMove\RobotArmMember.h"

//#define RobotArmState_DEBUG_MODE
#ifdef RobotArmState_DEBUG_MODE
#	define PRE Serial.print(F("RobotArmState : "))
#	define POST delay(2) // note missing ';'
#	define DEBUG1(x) PRE; Serial.println(x); POST
#	define DEBUG2(x,y) PRE; Serial.print(x); Serial.println(y); POST
#	define DEBUG3(f,xT,xF) PRE; if(f) { Serial.println(xT); } else { Serial.println(xF); } POST
#else
#	define DEBUG1(x)
#	define DEBUG2(x,y)
#	define DEBUG3(f,xT,xF)
#endif

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

void RobotArmState::setStoredPosition(NamedPosition posName, Position p)
{
	//PositionAngles pAngles = p.toPositionAngles();
	PositionAngles pAngles = positionToMemberAngles(p);

	switch (posName)
	{
	case NamedPosition::Center:
		posCenter.pAngles = pAngles;
		posCenter.pos = p;
		break;
	case NamedPosition::CenterSonar:
		posCenter.pAngles = pAngles;
		posCenter.pos = p;
		break;
	case NamedPosition::CenterSonar:
		posCenter.pAngles = pAngles;
		posCenter.pos = p;
		break;
	default:
		break;
	}
}

void RobotArmState::setStoredPosition(NamedPosition pName, Position p)
{
	switch (pName)
	{
	case NamedPosition::Center:

	}
}

void RobotArmState::goToPresetPosition(NamedPosition posName)
{
	boom2.smooth(boom2.getSafeServoAngle());	// replace with vertical lift function

	switch (posName)
	{
	case NamedPosition::Center:			// don't change claw angle
		boom1.slow(posCenter.boom1);
		turret.slow(posCenter.turret);
		boom2.slow(posCenter.boom2);
		break;
	case NamedPosition::CenterSonar:	// don't change claw angle
		boom1.slow(posCenterSonar.boom1);
		turret.slow(posCenterSonar.turret);
		boom2.slow(posCenterSonar.boom2);
		break;
	case NamedPosition::Rest:
		claw.write(posRest.claw);
		boom1.slow(posRest.boom1);
		turret.slow(posRest.turret);
		boom2.slow(posRest.boom2);
		break;
	default:
		break;
	}

	DEBUG2(F("Boom1 angle set to/at:  "), boom1.read());
	DEBUG2(F("Boom2 angle set to/at:  "), boom2.read());
	DEBUG2(F("Turret angle set to/at: "), turret.read());
	DEBUG2(F("Claw angle set to/at:   "), claw.read());
}

RobotArmMember& RobotArmState::getServo(RobotArmMember::Name name)
{
	switch (name)
	{
	case RobotArmMember::Name::Boom1: return boom1;
	case RobotArmMember::Name::Boom2: return boom2;
	case RobotArmMember::Name::Turret: return turret;
	case RobotArmMember::Name::Claw: return claw;
	default:
		return;
	}
}

int RobotArmState::getRadius()
{
	DEBUG2(F(": boom1 radius: "), boom1.getRadius());
	DEBUG2(F(": boom2 radius: "), boom2.getRadius());
	DEBUG2(F(": claw radius:  "), claw.getRadius());
	return boom1.getRadius() + boom2.getRadius() + claw.getRadius();
}

int RobotArmState::getHeight()
{
	DEBUG2(F(": boom1 height: "), boom1.getHeight());
	DEBUG2(F(": boom2 height: "), boom2.getHeight());
	DEBUG2(F(": claw height:  "), claw.getHeight());
	return boom1.getHeight() + boom2.getHeight() + claw.getHeight();
}

int RobotArmState::getTheta()
{
	return turret.getAngle();
}

RobotArmState::Position RobotArmState::getPosition()
{
	this->pos.set(getHeight(), getRadius(), getTheta());
	return pos;
}

void RobotArmState::goToPosition(Position p)
{
	verifyPosition(p);
}

void RobotArmState::goToPosition(int h, int r, int th)
{
	goToPosition(Position({ h, r, th }));
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
		memberList[i]->smooth(memberList[i]->getSafeServoAngle());

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
			memberList[i]->write(memberList[i]->getSafeServoAngle());
			memberList[i]->attach();
		}
		else
			memberList[i]->smooth(memberList[i]->getSafeServoAngle());
	}
}

void RobotArmState::verifyPosition(Position& position)
{
	//
}

RobotArmState::Position RobotArmState::memberAnglesToPosition(PositionAngles pAngles)
{
	//
}

RobotArmState::Position RobotArmState::memberAnglesToPosition(int b1, int b2, int turret)
{
	//
}

RobotArmState::PositionAngles RobotArmState::positionToMemberAngles(Position p)
{
	//
}
