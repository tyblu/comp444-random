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
	enum NamedPosition { 
		None, 
		CenterSonar, 
		Center, 
		Rest, 
		MaxHeight, 
		MinHeight, 
		MaxRadius, 
		MinRadius 
	};
	struct Position;	// declaration only for use in PositionAngles
	struct PositionAngles
	{
		PositionAngles() : boom1(0), boom2(0), turret(0) {}
		PositionAngles(int b1, int b2, int th) : boom1(b1), boom2(b2), turret(th) {}

		void set(int boom1, int boom2, int turret)
		{
			this->boom1 = boom1;
			this->boom2 = boom2;
			this->turret = turret;
		}

		bool equals(PositionAngles& pAngles)
		{
			return (this->boom1 == pAngles.boom1
				&& this->boom2 == pAngles.boom2
				&& this->turret == pAngles.turret);
		}

		Position& toPosition()				// not implemented
		{
			// convert
		}

		int boom1, boom2, turret;
	};

	struct Position
	{
		Position() : height(0), radius(0), theta(0) {}
		Position(int h, int r, int th) : height(h), radius(r), theta(th) {}

		void set(int h, int r, int th)
		{
			this->height = h;
			this->radius = r;
			this->theta = th;
		}

		bool equals(Position& pos)
		{
			return (this->height == pos.height
				&& this->radius == pos.radius
				&& this->theta == pos.radius);
		}

		bool equals(int h, int r, int th)
		{
			return equals(Position{ h, r, th });
		}

		PositionAngles toPositionAngles()	// not implemented
		{
			// convert
		}

		int height, radius, theta;
	};

	//// Maintains range 0-360 degrees.
	//struct Angle
	//{
	//	Angle& add(const Angle& other);
	//	Angle& operator+(const Angle& other);
	//	Angle& operator-(const Angle& other);
	//	Angle& add(Angle&& other);
	//	Angle& operator+(Angle&& other);
	//	Angle& operator-(Angle&& other);
	//	Angle& add(int angleInt);
	//	Angle& operator+(int angleInt);
	//	Angle& operator-(int angleInt);
	//	Angle& operator++();
	//	Angle& operator--();
	//	// continues... http://en.cppreference.com/w/cpp/language/operators
	//
	//	bool equals(const Angle& other);
	//	bool operator==(const Angle& other);
	//
	//	int value;
	//};

	class RobotArmMemberIterator {
	public:
		RobotArmMemberIterator() : iterator(0), iteratorHasWrapped(false) {}

		RobotArmMember::Name next()
		{
			iterator++;
			iteratorHasWrapped = false;
			return current();
		}

		RobotArmMember::Name current()
		{
			switch (iterator)
			{
			case 0:	return RobotArmMember::Name::Boom1;
			case 1: return RobotArmMember::Name::Boom2;
			case 2: return RobotArmMember::Name::Turret;
			case 3: return RobotArmMember::Name::Claw;
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

	RobotArmMember& getServo(RobotArmMember::Name name);

	int getRadius();	// endeffector working radius
	int getHeight();	// endeffector working height
	int getTheta();		// turret angle
	Position getPosition();

	void setBoom2MinMap(nsTyblu::Pair pairArray[], uint8_t count);
	void setStoredPosition(NamedPosition posName, Position p);

	Position memberAnglesToPosition(PositionAngles pAngles);
	Position memberAnglesToPosition(int b1, int b2, int turret);
	PositionAngles positionToMemberAngles(Position p);


	bool isServoPowerOn();
	void servoPowerOn();
	void servoPowerOff();

	void sweep();
	void attachSafe();

	void goToPosition(Position p);
	void goToPosition(int height, int radius, int theta);
	void goToPresetPosition(NamedPosition namedPos);

	void verifyPosition(Position& pos);

private:
	EndEffectorState endEffectorState;
	const uint8_t pwrEnablePin, pwrFeedbackPin;
	RobotArmMember& boom1, boom2, turret, claw;
	RobotArmMember* memberList[4];
	nsTyblu::InterpolatedMap boom2min;
};

#endif // RobotArmState_h