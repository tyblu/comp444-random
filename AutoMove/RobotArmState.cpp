#include "RobotArmState.h"
#include "C:\Users\tyblu\Documents\repos\comp444-random\AutoMove\RobotArmMember.h"

//#define RobotArmState_DEBUG_MODE
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
//#	define RobotArmState_DEBUG_MODE_LEVEL1
#   ifdef RobotArmState_DEBUG_MODE_LEVEL1
#	   define DEBUG0_LV1(x) DEBUG0(x)
#	   define DEBUG00_LV1(x) DEBUG00(x)
#	   define DEBUG1_LV1(x) DEBUG1(x)
#	   define DEBUG10_LV1(x) DEBUG10(x)
#	   define DEBUG2_LV1(x,y) DEBUG2(x,y)
#	   define DEBUG20_LV1(x,y) DEBUG20(x,y)
#	   define DEBUG3_LV1(f,xT,xF) DEBUG3(f,xT,xF)
#	   define DEBUG30_LV1(f,xT,xF) DEBUG30(f,xT,xF)
#   else
#	   define DEBUG0_LV1(x)
#	   define DEBUG00_LV1(x)
#	   define DEBUG1_LV1(x)
#	   define DEBUG10_LV1(x)
#	   define DEBUG2_LV1(x,y)
#	   define DEBUG20_LV1(x,y)
#	   define DEBUG3_LV1(f,xT,xF)
#	   define DEBUG30_LV1(f,xT,xF)
#   endif

#define DIR RobotArmState::Direction

#define POWER_DELAY_MS 200						// power up/down delay
#define SERVO_PWR_FEEDBACK_ANALOGREAD_POINTS 20	// < uint8_t max and even
#define GOTOPOSITION_TIMEOUT_MS 5000			// allowed time to seek to a position
#define PRESET_POSITIONS_DELAY_MS 300			// delay between positions

RobotArmState::RobotArmState(
	uint8_t pwrEnablePin,
	uint8_t pwrFeedbackPin,
	RobotArmMember& boom1,
	RobotArmMember& boom2,
	RobotArmMember& turret,
	RobotArmMember& claw,
	PositionVector preset[PRESET_POSITIONS_COUNT],
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
	, posCup(preset[7].h, preset[7].r, preset[7].th)
	, posA(preset[8].h, preset[8].r, preset[8].th)
	, posB(preset[9].h, preset[9].r, preset[9].th)
	, posC(preset[10].h, preset[10].r, preset[10].th)
	, posD(preset[11].h, preset[11].r, preset[11].th)
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
	DEBUG10_LV1(F("Going to position "));

	switch (posName)
	{
	case NamedPosition::Center:
		DEBUG00_LV1(Serial.println(F("[Center] ... ")));
		goToPosition(posCenter);
		break;
	case NamedPosition::CenterSonar:
		DEBUG00_LV1(Serial.println(F("[CenterSonar] ... ")));
		goToPosition(posCenterSonar);
		break;
	case NamedPosition::Rest:
		DEBUG00_LV1(Serial.println(F("[Rest] ... ")));
		goToPosition(posRest);
		break;
	case NamedPosition::MaxHeight:
		DEBUG00_LV1(Serial.println(F("[MaxHeight] ... ")));
		goToPosition(posMaxHeight);
		break;
	case NamedPosition::MinHeight:
		DEBUG00_LV1(Serial.println(F("[MinHeight] ... ")));
		goToPosition(posMinHeight);
		break;
	case NamedPosition::MaxRadius:
		DEBUG00_LV1(Serial.println(F("[MaxRadius] ... ")));
		goToPosition(posMaxRadius);
		break;
	case NamedPosition::MinRadius:
		DEBUG00_LV1(Serial.println(F("[MinRadius] ... ")));
		goToPosition(posMinRadius);
		break;
	case NamedPosition::Cup:
		DEBUG00_LV1(Serial.println(F("[Cup] ... ")));
		goToPosition(posCup);
		break;
	case NamedPosition::A:
		DEBUG00_LV1(Serial.println(F("[A] ... ")));
		goToPosition(posA);
		break;
	case NamedPosition::B:
		DEBUG00_LV1(Serial.println(F("[B] ... ")));
		goToPosition(posB);
		break;
	case NamedPosition::C:
		DEBUG00_LV1(Serial.println(F("[C] ... ")));
		goToPosition(posC);
		break;
	case NamedPosition::D:
		DEBUG00_LV1(Serial.println(F("[D] ... ")));
		goToPosition(posD);
		break;
	default:
		DEBUG1(F("\nERROR: No switch-case for given NamedPosition!"));
		break;
	}
	DEBUG0_LV1(F("done."));
}

void RobotArmState::goToPosition(PositionVector p)	// no verification
{
	int angle1 = boom1.getServo()->read();
	int angle2 = boom2.getServo()->read();

	long timeout = millis() + GOTOPOSITION_TIMEOUT_MS;
	long timeLeft = GOTOPOSITION_TIMEOUT_MS;
	while (pos.delta(p) > 2 && timeLeft > 0)	// error up to 1mm or 1deg or combo of two
	{
		if (p.h > pos.h && angle2 + 1 <= boom2.getMaxAngle())	// raise boom2
		{
			do {
				boom2.slow(++angle2);
				pos.updateAll();
				timeLeft = timeout - millis();
			} while (p.h > pos.h && angle2+1 < boom2.getMaxAngle() && timeLeft > 0);
		}

		if (p.r < pos.r && angle1 - 1 >= boom1.getMinAngle())	// reduce radius with boom1
		{
			do {
				boom1.slow(--angle1);
				pos.updateAll();
				timeLeft = timeout - millis();
			} while (p.r < pos.r && angle1-1 > boom1.getMinAngle() && timeLeft > 0);
		}

		if (p.th != pos.th)	// rotate to position
			turret.change(turret.toServoAngle(p.th - pos.th));

		// seek target radius and height simultaneously
		if (p.h > pos.h && angle2 + 1 <= boom2.getMaxAngle())	// up
			boom2.slow(++angle2);
		if (p.r < pos.r && angle1 - 1 >= boom1.getMinAngle())	// in
			boom1.slow(--angle1);
		if (p.h < pos.h && angle2 - 1 >= boom2.getMinAngle())	// down
			boom2.slow(--angle2);
		if (p.r > pos.r && angle1 + 1 <= boom1.getMaxAngle())	// out
			boom1.slow(++angle1);

		pos.updateAll();
		timeLeft = timeout - millis();
	}

	DEBUG3(timeLeft < 0, F("goToPos() timeout"), F("goToPos() finished on time"));

	DEBUG10(F("Pos. "));
	DEBUG00(printPosition(p));
	DEBUG00(Serial.print(F(" found at boom angles {")));
	DEBUG00(Serial.print(angle1));
	DEBUG00(Serial.write(','));
	DEBUG00(Serial.print(angle2));
	DEBUG00(Serial.print(F("} for actual pos. ")));
	DEBUG00(printPosition(pos));
	DEBUG00(Serial.print(F(", diff=")));
	DEBUG0(Serial.print(pos.delta(p)));
}

void printPosition(PositionVector& p)	// move to PositionVector class
{
	Serial.write('(');
	Serial.print(p.h);
	Serial.write(',');
	Serial.print(p.r);
	Serial.write(',');
	Serial.print(p.th);
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
	return (sum > SERVO_PWR_FEEDBACK_ANALOGREAD_POINTS * 1023 / 2);
}

void RobotArmState::servoPowerOn()
{
	if (digitalRead(pwrEnablePin) == HIGH)
		return;		// already on, just return

	digitalWrite(pwrEnablePin, HIGH);
	delay(POWER_DELAY_MS);

	this->getPositionVector()->updateAll();
}

void RobotArmState::servoPowerOff()
{
	digitalWrite(pwrEnablePin, LOW);

	for (uint8_t i = 0; i < 4; i++)
		memberList[i]->getServo()->detach();

	delay(POWER_DELAY_MS);
}

void RobotArmState::sweep()
{
	for (uint8_t i = 0; i < 4; i++)
		memberList[i]->sweep();
}

void RobotArmState::sweepPresetPositions()
{
	PositionVector * pos[PRESET_POSITIONS_COUNT] =
	{
		&posCenterSonar,
		&posCenter,
		&posRest,
		&posCup,
		&posMaxHeight, &posMinHeight, &posMaxRadius, &posMinRadius,
		&posA, &posB, &posC, &posD
	};

	for (uint8_t i = 0; i < PRESET_POSITIONS_COUNT; i++)
	{
		goToPosition(*pos[i]);
		delay(PRESET_POSITIONS_DELAY_MS);
	}
}

void RobotArmState::attach()
{
	for (uint8_t i = 0; i < 4; i++)
		if (!memberList[i]->getServo()->attached())
			memberList[i]->attach();
}

void RobotArmState::attachSafe()
{
	for (uint8_t i = 0; i < 4; i++)
		memberList[i]->safe();
	attach();
}

void RobotArmState::init()
{
	/* Ensure servos are within working ranges. */
	for (uint8_t i = 0; i < 4; i++)
	{
		int readAngle = memberList[i]->getServo()->read();
		if (readAngle < memberList[i]->getMinAngle())
			memberList[i]->getServo()->write(memberList[i]->getMinAngle());
		if (readAngle > memberList[i]->getMaxAngle())
			memberList[i]->getServo()->write(memberList[i]->getMaxAngle());
	}

	//determineExtents();	// currently takes too long, not important enough
}

/* This currently takes about 14 minutes!! */
void RobotArmState::determineExtents()
{
	DEBUG2(F("determineExtents() timestamp: "), millis());
	DEBUG10(F("Determining movement extents."));

	int maxRadius = INT_MIN;
	int minRadius = INT_MAX;
	int maxHeight = INT_MIN;
	int minHeight = INT_MAX;

//	PositionVector &p1 = *(boom1.getPositionVector());
//	PositionVector &p2 = *(boom2.getPositionVector());
//	PositionVector &p3 = *(claw.getPositionVector());

//	int max1 = boom1.getMaxAngle();
//	int min1 = boom1.getMinAngle();
//	int max2 = boom2.getMaxAngle();
//	int min2 = boom2.getMinAngle();
//	int max3 = claw.getMaxAngle();
//	int min3 = claw.getMinAngle();

	int radius, height;
	for (int a1 = boom1.getMinAngle(); a1 <= boom1.getMaxAngle(); a1++)
	{
		for (int a2 = boom2.getMinAngle(); a2 <= boom2.getMaxAngle(); a2++)
		{
			for (int a3 = claw.getMinAngle(); a3 <= claw.getMaxAngle(); a3++)
			{
				radius = boom1.getPositionVector()->getRadius(boom1.toPhysicalAngle(a1))
					+ boom2.getPositionVector()->getRadius(boom2.toPhysicalAngle(a2))
					+ claw.getPositionVector()->getRadius(claw.toPhysicalAngle(a3));

				height = boom1.getPositionVector()->getHeight(boom1.toPhysicalAngle(a1))
					+ boom2.getPositionVector()->getHeight(boom2.toPhysicalAngle(a2))
					+ claw.getPositionVector()->getHeight(claw.toPhysicalAngle(a3));

//				radius = p1.getRadius(boom1.toPhysicalAngle(a1))
//					+ p2.getRadius(boom2.toPhysicalAngle(a2))
//					+ p3.getRadius(claw.toPhysicalAngle(a3));
//
//				height = p1.getHeight(boom1.toPhysicalAngle(a1))
//					+ p2.getHeight(boom2.toPhysicalAngle(a2))
//					+ p3.getHeight(claw.toPhysicalAngle(a3));

//				DEBUG20_LV1(F("radius: "), radius);
//				DEBUG00_LV1(Serial.print(F(", height: ")));
//				DEBUG00_LV1(Serial.print(height));
//				DEBUG00_LV1(Serial.print(F(" at [")));
//				DEBUG00_LV1(Serial.print(a1));
//				DEBUG00_LV1(Serial.write(','));
//				DEBUG00_LV1(Serial.print(a2));
//				DEBUG00_LV1(Serial.write(','));
//				DEBUG00_LV1(Serial.print(a3));
//				DEBUG00_LV1(Serial.println("]"));

				if (radius > maxRadius)
				{
					this->posMaxRadius.set(height, radius, posMaxRadius.th);
					maxRadius = radius;
					DEBUG10_LV1(F("posMaxRadius changed to: "));
					DEBUG0_LV1(printPosition(posMaxRadius));
				}

				if (height > maxHeight)
				{
					this->posMaxHeight.set(height, radius, posMaxHeight.th);
					maxHeight = height;
					DEBUG10_LV1(F("posMaxHeight changed to: "));
					DEBUG0_LV1(printPosition(posMaxHeight));
				}

				if (radius < minRadius)
				{
					this->posMinRadius.set(height, radius, posMinRadius.th);
					minRadius = radius;
					DEBUG10_LV1(F("posMinRadius changed to: "));
					DEBUG0_LV1(printPosition(posMinRadius));
				}

				if (height < minHeight)
				{
					this->posMinHeight.set(height, radius, posMaxRadius.th);
					minHeight = height;
					DEBUG10_LV1(F("posMinHeight changed to: "));
					DEBUG0_LV1(printPosition(posMinHeight));
				}
			}
		}
		DEBUG00(Serial.write('.'));
	}

	DEBUG00(Serial.println());

	DEBUG2(F("determineExtents() timestamp: "), millis());

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