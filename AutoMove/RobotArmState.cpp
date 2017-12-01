#include "RobotArmState.h"
#include "C:\Users\tyblu\Documents\repos\comp444-random\AutoMove\RobotArmMember.h"

#define RobotArmState_DEBUG_MODE
#ifdef RobotArmState_DEBUG_MODE
#	define PRE Serial.print(F("RobotArmState : "))
#	define POST delay(2) // note missing ';'
#	define DEBUG0(x) (x); Serial.println(); POST
#	define DEBUG00(x) (x); POST
#	define DEBUG1(x) PRE; Serial.println(x); POST
#	define DEBUG10(x) PRE; Serial.print(x); POST
#	define DEBUG2(x,y) PRE; Serial.print(x); Serial.println(y); POST
#	define DEBUG20(x,y) PRE; Serial.print(x); Serial.print(y); POST
#	define DEBUG3(f,xT,xF) PRE; if(f) { Serial.println(xT); } else { Serial.println(xF); } POST
#	define DEBUG30(f,xT,xF) PRE; if(f) { Serial.print(xT); } else { Serial.print(xF); } POST
#else
#	define DEBUG0(x)
#	define DEBUG00(x)
#	define DEBUG1(x)
#	define DEBUG10(x)
#	define DEBUG2(x,y)
#	define DEBUG20(x,y)
#	define DEBUG3(f,xT,xF)
#	define DEBUG30(f,xT,xF)
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
		DEBUG1(F("ERROR: No switch-case for given NamedPosition!"));
		break;
	}

	DEBUG2(F("Boom1 angle set to/at:  "), boom1.getServo()->read());
	DEBUG2(F("Boom2 angle set to/at:  "), boom2.getServo()->read());
	DEBUG2(F("Turret angle set to/at: "), turret.getServo()->read());
	DEBUG2(F("Claw angle set to/at:   "), claw.getServo()->read());
}

void RobotArmState::goToPosition(PositionVector p)	// no verification
{
	byte step = 0;
	int angle1, angle2;
	PositionVector posNext = { 0, 0, 0 };

	//unsigned long timeout = millis() + GOTOPOSITION_TIMEOUT_MS;
	while (!pos.equals(p)/* && millis() < timeout*/)
	{
		if (p.h > pos.h)	// raise boom2
		{
			angle2 = boom2.getServo()->read();
			do {
				posNext.h = boom2.getPositionVector()->getHeight(boom2.toPhysicalAngle(angle2));
				angle2++;
			} while (posNext.h < p.h && angle2 < boom2.getMaxAngle());
			boom2.slow(--angle2);
			pos.update();
		}

		if (p.r < pos.r)	// reduce radius with boom1
		{
			angle1 = boom1.getServo()->read();
			do {
				posNext.r = boom1.getPositionVector()->getRadius(boom1.toPhysicalAngle(angle1));
				angle1++;
			} while (posNext.r < p.r && angle1 < boom1.getMaxAngle());
		}

		if (pos.th != p.th)	// rotate to position
			turret.slow(p.th);

		// seek target radius and height simultaneously
		while (pos.delta(p) > 6)	// error up to 6mm or 6deg or combo of two
		{
			if (p.h > pos.h && angle2 < boom2.getMaxAngle())
				boom2.slow(++angle2);
			if (p.r < pos.r && angle1 < boom1.getMaxAngle())
				boom1.slow(++angle1);
			if (p.h < pos.h && angle2 > boom2.getMinAngle())
				boom2.slow(--angle2);
			if (p.r > pos.r && angle1 > boom1.getMinAngle())
				boom1.slow(--angle1);
		}

		DEBUG10(F("Pos. "));
		DEBUG00(printPosition(p));
		DEBUG10(F(" found at boom angles {"));
		DEBUG10(angle1);
		DEBUG10(',');
		DEBUG10(angle2);
		DEBUG10(F("} for actual pos. "));
		DEBUG00(printPosition(pos));
		DEBUG10('.');
	}
}

void printPosition(PositionVector& pos)	// move to PositionVector class
{
	Serial.write('(');
	Serial.print(pos.h);
	Serial.write(',');
	Serial.print(pos.r);
	Serial.write(',');
	Serial.print(pos.th);
	Serial.write(')');
	delay(4);
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
	for (uint8_t i = 0; i < 4; i++)
		memberList[i]->sweep();
}

void RobotArmState::attachSafe()
{
	for (uint8_t i = 0; i < 4; i++)
		memberList[i]->safe();

	for (uint8_t i = 0; i < 4; i++)
		if (!memberList[i]->getServo()->attached())
			memberList[i]->attach();
}

void RobotArmState::init()
{
	determineExtents();
}

void RobotArmState::determineExtents()
{
	int maxRadius = INT_MIN;
	int minRadius = INT_MAX;
	int maxHeight = INT_MIN;
	int minHeight = INT_MAX;

	DEBUG2(F("boom1.getMinAngle():  "), boom1.getMinAngle());
	DEBUG2(F("boom1.getMaxAngle():  "), boom1.getMaxAngle());
	DEBUG2(F("boom2.getMinAngle():  "), boom2.getMinAngle());
	DEBUG2(F("boom2.getMaxAngle():  "), boom2.getMaxAngle());
//	DEBUG2(F(".[1]->getMinAngle():  "), memberList[1]->getMinAngle());
//	DEBUG2(F(".[1]->getMaxAngle():  "), memberList[1]->getMaxAngle());
	DEBUG2(F("turret.getMinAngle(): "), turret.getMinAngle());
	DEBUG2(F("turret.getMaxAngle(): "), turret.getMaxAngle());
	DEBUG2(F("claw.getMinAngle():   "), claw.getMinAngle());
	DEBUG2(F("claw.getMaxAngle():   "), claw.getMaxAngle());

	int radius, height;
	for (int a1 = boom1.getMinAngle(); a1 < boom1.getMaxAngle(); a1++)
	{
		Serial.println(F("INSIDE FIRST FOR LOOP")); delay(2);
		for (int a2 = boom2.getMinAngle(); a2 < boom2.getMaxAngle(); a2++)
		{
			Serial.println(F("INSIDE SECOND FOR LOOP")); delay(2);
			for (int a3 = claw.getMinAngle(); a3 < claw.getMaxAngle(); a3++)
			{
				Serial.println(F("INSIDE THIRD FOR LOOP")); delay(2);
				radius = boom1.getPositionVector()->getRadius(boom1.toPhysicalAngle(a1))
					+ boom2.getPositionVector()->getRadius(boom2.toPhysicalAngle(a2))
					+ claw.getPositionVector()->getRadius(claw.toPhysicalAngle(a3));

				height = boom1.getPositionVector()->getHeight(boom1.toPhysicalAngle(a1))
					+ boom2.getPositionVector()->getHeight(boom2.toPhysicalAngle(a2))
					+ claw.getPositionVector()->getHeight(claw.toPhysicalAngle(a3));

				DEBUG20(F("radius: "), radius);
				DEBUG00(Serial.print(F(", height: ")));
				DEBUG00(Serial.print(height));
				DEBUG00(Serial.print(F(" at [")));
				DEBUG00(Serial.print(a1));
				DEBUG00(Serial.write(','));
				DEBUG00(Serial.print(a2));
				DEBUG00(Serial.write(','));
				DEBUG00(Serial.print(a3));
				DEBUG00(Serial.println("]"));

				if (radius > maxRadius)
				{
					this->posMaxRadius.set(height, radius, posMaxRadius.th);
					maxRadius = radius;
					DEBUG10(F("posMaxRadius changed to: "));
					DEBUG0(printPosition(posMaxRadius));
				}

				if (height > maxHeight)
				{
					this->posMaxHeight.set(height, radius, posMaxHeight.th);
					maxHeight = height;
					DEBUG10(F("posMaxHeight changed to: "));
					DEBUG0(printPosition(posMaxHeight));
				}

				if (radius < minRadius)
				{
					this->posMinRadius.set(height, radius, posMinRadius.th);
					minRadius = radius;
					DEBUG10(F("posMinRadius changed to: "));
					DEBUG0(printPosition(posMinRadius));
				}

				if (height < minHeight)
				{
					this->posMinHeight.set(height, radius, posMaxRadius.th);
					minHeight = height;
					DEBUG10(F("posMinHeight changed to: "));
					DEBUG0(printPosition(posMinHeight));
				}
			}
		}
	}

	DEBUG10(F("posMinHeight: "));
	DEBUG0(printPosition(posMinHeight));
	DEBUG10(F("posMaxHeight: "));
	DEBUG0(printPosition(posMaxHeight));
	DEBUG10(F("posMinRadius: "));
	DEBUG0(printPosition(posMinRadius));
	DEBUG10(F("posMaxRadius: "));
	DEBUG0(printPosition(posMaxRadius));
}

EndEffectorPositionVector * RobotArmState::getPositionVector()
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