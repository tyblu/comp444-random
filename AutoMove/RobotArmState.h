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
#include "limits.h"

#define INTERP_PAIRS_MAX_COUNT 10

namespace nsTyblu
{
	struct Pair 
	{ 
		Pair(int k, int v) : key(k), val(v) {}
		Pair() : key(INT_MAX), val(INT_MAX) {}
		int key, val;
	};
	const static Pair nullPair = Pair(INT_MAX, INT_MAX);

	class Map 
	{
	public:
		Map() : pairs(), top(0) {}

		Pair get(int key)
		{
			for (uint8_t i = 0; i < top; i++)
				if (pairs[i].key == key)
					return pairs[i];
			return nullPair;
		}

		bool containsKey(int key)
		{
			for (uint8_t i = 0; i < top; i++)
				if (pairs[i].key == key)
					return true;
			return false;
		}

		bool isEmpty()
		{
			return (top == 0);
		}

		// returns k-v pair with largest key less than or equal to given key
		Pair lowerBound(int key)
		{
			for (uint8_t i = 1; i < top; i++)
				if (pairs[i].key > key)
					return pairs[i-1];
			return nullPair;
		}

		Pair upperBound(int key)
		{
			for (uint8_t i = 1; i < top; i++)
				if (pairs[i].key > key)
					return pairs[i];
			return nullPair;
		}

		void put(int key, int val)
		{
			if (top >= INTERP_PAIRS_MAX_COUNT)
				return;
			pairs[top].key = key;
			pairs[top].val = val;
			top++;
			insertionSortPairsByKey(pairs, top);
		}

	private:
		Pair pairs[INTERP_PAIRS_MAX_COUNT];
		uint8_t top;

		static void insertionSortPairsByKey(Pair arr[], uint8_t len)
		{
			for (uint8_t i = 1; i < len; ++i)
			{
				Pair tmp = arr[i];
				uint8_t j = i;
				while (j > 0 && tmp.key < arr[j - 1].key)
				{
					arr[j] = arr[j - 1];
					--j;
				}
				arr[j] = tmp;
			}
		}
	};

	class InterpolatedMap : public Map
	{
	public:
		InterpolatedMap() : Map() {}

		// Returns interpolated value between those with keys greater and less
		// than the given key.
		int get(int key)
		{
			if (Map::containsKey(key))
				return Map::get(key).val;

			Pair lower = Map::lowerBound(key);
			Pair upper = Map::upperBound(key);

			return intLInterp(key, lower.key, lower.val, upper.key, upper.val);
		}

	private:
		// Linear interpolation for integers. 
		// Uses rounding to nearest.
		// Repeat data points, equal (x0,x1), returns average of (y0,y1).
		static int intLInterp(int x, int x0, int y0, int x1, int y1)
		{
			if ((x1 - x0) == 0)
				return (y0 + y1 + 1) / 2;
			return y0 + ((x - x0) * (y1 - y0) + (x1 - x0) / 2) / (x1 - x0);
		}
	};
}

class RobotArmState
{
public:
	enum EndEffectorState { P45Deg, P00Deg, N45Deg, N90Deg, N135Deg } ;
	struct Position 
	{
		void set(int boom1, int boom2, int turret, int claw)
		{
			this->boom1 = boom1;
			this->boom2 = boom2;
			this->turret = turret;
			this->claw = claw;
		}
		int boom1, boom2, turret, claw;
	};
	enum NamedPosition { CenterSonar, Center, Rest };

	class RobotArmMemberIterator {
	public:
		RobotArmMemberIterator() : iterator(0), iteratorHasWrapped(false) {}

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
		
		void restart()
		{
			iterator = 0;
			iteratorHasWrapped = false;
		}

		bool isFinished()
		{
			return iteratorHasWrapped;
		}

	private:
		uint8_t iterator;
		bool iteratorHasWrapped;
	} list;

	RobotArmState(EndEffectorState endEffectorState, 
		uint8_t pwrEnablePin,
		uint8_t pwrFeedbackPin,
		RobotArmMember& boom1,
		RobotArmMember& boom2,
		RobotArmMember& turret,
		RobotArmMember& claw
	);

	RobotArmMember& getServo(RobotArmMember::ServoName name);

	uint16_t getRadius();	// endeffector working radius
	uint16_t getHeight();	// endeffector working height
	uint16_t getTheta();	// turret angle

	void setBoom2MinMap(nsTyblu::Pair pairArray[], uint8_t count);
	void RobotArmState::setStoredPosition(NamedPosition posName,
		int a, int b, int c, int d);

	bool isServoPowerOn();
	void servoPowerOn();
	void servoPowerOff();

	void sweep();
	void attachSafe();

	void goToPos(NamedPosition pos);

private:
	EndEffectorState endEffectorState;
	const uint8_t pwrEnablePin, pwrFeedbackPin;
	RobotArmMember& boom1, boom2, turret, claw;
	RobotArmMember* memberList[4];
	nsTyblu::InterpolatedMap boom2min;
	Position posCenterSonar, posCenter, posRest, posOff;
};

#endif // RobotArmState_h