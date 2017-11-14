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

#define DIR RobotArmState::Direction

#define SERVO_PWR_FEEDBACK_ANALOGREAD_POINTS 20	// < uint8_t max and even
#define GOTOPOSITION_STEPS 3
//#define GOTO_NUM 2		// initial boom2 raise of NUM/DENOM fraction of delta
//#define GOTO_DENOM 3

RobotArmState::RobotArmState(
	uint8_t pwrEnablePin,
	uint8_t pwrFeedbackPin,
	RobotArmMember& boom1,
	RobotArmMember& boom2,
	RobotArmMember& turret,
	RobotArmMember& claw,
	PositionVector preset[7],
	Pair pairArray[BOOM2_MIN_PAIR_ARRAY_COUNT]
)
	: pwrEnablePin(pwrEnablePin)
	, pwrFeedbackPin(pwrFeedbackPin)
	, boom1(boom1)
	, boom2(boom2)
	, turret(turret)
	, claw(claw)
	, memberList()
	, posCenterSonar(preset[0].h, preset[0].r, preset[0].th)
	, posCenter(preset[1].h, preset[1].r, preset[1].th)
	, posRest(preset[2].h, preset[2].r, preset[2].th)
	, posMaxHeight(preset[3].h, preset[3].r, preset[3].th)
	, posMinHeight(preset[4].h, preset[4].r, preset[4].th)
	, posMaxRadius(preset[5].h, preset[5].r, preset[5].th)
	, posMinRadius(preset[6].h, preset[6].r, preset[6].th)
	, interpMapBoom2Min()
	, pos(boom1, boom2, turret, claw)
{
	digitalWrite(pwrEnablePin, LOW);			// ensure servo power is off
	pinMode(pwrEnablePin, OUTPUT);
	pinMode(pwrFeedbackPin, INPUT);

	memberList[0] = &boom1;
	memberList[1] = &boom2;
	memberList[2] = &turret;
	memberList[3] = &claw;

	interpMapBoom2Min.putAll(pairArray, BOOM2_MIN_PAIR_ARRAY_COUNT);
}



void RobotArmState::goToPosition(NamedPosition posName)
{
	switch (posName)
	{
	case NamedPosition::Center:
		goToPosition(posCenter);
		break;
	case NamedPosition::CenterSonar:
		goToPosition(posCenterSonar);
		break;
	case NamedPosition::Rest:
		goToPosition(posRest);
		break;
	case NamedPosition::MaxHeight:
		goToPosition(posMaxHeight);
		break;
	case NamedPosition::MinHeight:
		goToPosition(posMinHeight);
		break;
	case NamedPosition::MaxRadius:
		goToPosition(posMaxRadius);
		break;
	case NamedPosition::MinRadius:
		goToPosition(posMinRadius);
		break;
	default:
		DEBUG1("ERROR: No switch-case for given NamedPosition!");
		break;
	}

	DEBUG2(F("Boom1 angle set to/at:  "), boom1.read());
	DEBUG2(F("Boom2 angle set to/at:  "), boom2.read());
	DEBUG2(F("Turret angle set to/at: "), turret.read());
	DEBUG2(F("Claw angle set to/at:   "), claw.read());
}

void RobotArmState::goToPosition(PositionVector p)	// no verification
{
	byte step = 0;
	int angle;
	PositionVector posNext = { 0, 0, 0 };

	//unsigned long timeout = millis() + GOTOPOSITION_TIMEOUT_MS;
	while (!pos.equals(p)/* && millis() < timeout*/)
	{
		if (p.h > pos.h)	// raise boom2
		{
			angle = boom2.getAngle();
			do {
				posNext.h = boom2.getPositionVector()->getHeight(angle);
				angle++;
			} while (posNext.h < p.h && angle < boom2.getMaxAngle());
			boom2.slow(--angle);
			pos.update();
		}

		if (p.r < pos.r)	// reduce radius with boom1
		{
			angle = boom1.getAngle();
			do {
				posNext.r = boom1.getPositionVector()->getRadius(angle);
				angle++;
			} while (posNext.r < p.r && angle < boom1.getMaxAngle());
		}

		if (pos.th != p.th)
			turret.slow(p.th);
	}
}

void RobotArmState::goToPosition(int h, int r, int th)
{
	PositionVector posTarget = { h, r, th };
	verifyPosition(posTarget);
	goToPosition(posTarget);
}

void RobotArmState::constrainedMove(RobotArmState::Direction dir, int value)
{
	//unsigned long timeout;
	PositionVector posTarget(0, 0, 0);

	switch (dir)
	{
	case DIR::Vertical:
		posTarget.add(pos.h + value, pos.r, pos.th);
		verifyPosition(posTarget);
		//timeout = millis() + CONSTRAINED_MOVE_TIMOUT_MS;
		while (!pos.equals(posTarget)/* && millis() < timetout */)
			goToPosition(pos.h + ((value < 0) ? -1 : 1), pos.r, pos.th);
		break;
	case DIR::Radial:
		posTarget.add(pos.h, pos.r + value, pos.th);
		//timeout = millis() + CONSTRAINED_MOVE_TIMEOUT_MS;
		while (!pos.equals(posTarget)/* && millis() < timeout */)
			goToPosition(pos.h, pos.r + ((value < 0) ? -1 : 1), pos.th);
		break;
	default:
		DEBUG1("constrainedMove ERROR: No switch-case for given RAS::Direction!");
		break;
	}
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
	int initialAngles[4];
	for (uint8_t i = 0; i < 4; i++)
		initialAngles[i] = memberList[i]->getAngle();

	for (uint8_t i = 0; i < 4; i++)
		memberList[i]->sweep();

	for (uint8_t i = 0; i < 4; i++)
		memberList[i]->slow(initialAngles[i]);
}

void RobotArmState::attachSafe()
{
	for (uint8_t i = 0; i < 4; i++)
		memberList[i]->safe();

	for (uint8_t i = 0; i < 4; i++)
		if (!memberList[i]->getServo()->attached())
			memberList[i]->attach();
}

PositionVector * RobotArmState::getPositionVector()
{
	return &pos;
}

bool RobotArmState::verifyPosition(PositionVector& p)	// needs work
{
	if (p.r > posMaxRadius.r 
		|| p.r < posMinRadius.r
		|| p.h > posMaxHeight.h
		|| p.h < posMinHeight.h)
		return false;
	return true;
}

PositionVector RobotArmState::memberAnglesToPosition(int b1, int b2, int th)
{
	return { boom1.getPositionVector()->getHeight(b1)
		+ boom2.getPositionVector()->getHeight(b2)
		, boom1.getPositionVector()->getRadius(b1)
		+ boom2.getPositionVector()->getRadius(b2)
		, th };
}