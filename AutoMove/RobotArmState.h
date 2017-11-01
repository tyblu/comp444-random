/*
* RobotArmState.h
*
*  Created on: Oct 31, 2017
*      Author: Tyler Lucas
*/

#ifndef RobotArmState_h
#define RobotArmState_h

#include "C:\Users\tyblu\Documents\repos\comp444-random\AutoMove\RobotArmMember.h"
#include "inttypes.h"

class RobotArmState
{
public:
	enum EndEffectorState { P45Deg, P00Deg, N45Deg, N90Deg, N135Deg } ;

	class RobotArmMemberIterator {
	public:
		RobotArmMember::ServoName next()
		{
			iterator++;
			iteratorHasWrapped = false;
			return current();
		}
		RobotArmMember::ServoName current()
		{
			switch (iterator)
			{
			case 0:	return RobotArmMember::ServoName::Boom1;
			case 1: return RobotArmMember::ServoName::Boom2;
			case 2: return RobotArmMember::ServoName::Turret;
			case 3: return RobotArmMember::ServoName::Claw;
			default:
				iterator = 0;
				iteratorHasWrapped = true;
				return current();
			}
		}
		void restart() { iterator = 0; }
		bool isFinished() { return iteratorHasWrapped; }

		RobotArmMemberIterator() : iterator(0), iteratorHasWrapped(false) {}

	private:
		uint8_t iterator;
		bool iteratorHasWrapped;
	} list;

	RobotArmState(EndEffectorState endEffectorState,
		RobotArmMember& boom1,
		RobotArmMember& boom2,
		RobotArmMember& turret,
		RobotArmMember& claw
	);

	RobotArmMember & getServo(RobotArmMember::ServoName name);

	uint16_t getRadius();	// endeffector working radius
	uint16_t getHeight();	// endeffector working height
	uint16_t getTheta();	// turret angle

private:
	EndEffectorState endEffectorState;
	RobotArmMember & boom1, boom2, turret, claw;
};

#endif // RobotArmState_h