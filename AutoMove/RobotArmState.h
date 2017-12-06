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

#define CLASS_MAP_INTERP_PAIRS_MAX_COUNT 10
#define BOOM2_MIN_PAIR_ARRAY_COUNT 10
#define PRESET_POSITIONS_COUNT 12

void printPosition(PositionVector& pos);

struct Pair { int key, val; };
const static Pair nullPair = Pair{ INT_MAX, INT_MAX };

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
		if (top >= CLASS_MAP_INTERP_PAIRS_MAX_COUNT)
			return;
		pairs[top].key = key;
		pairs[top].val = val;
		top++;
		insertionSortPairsByKey(pairs, top);
	}

	void putAll(Pair pairArray[], int count)
	{
		for (uint8_t i = 0; i < count; i++)
			this->put(pairArray[i].key, pairArray[i].val);
	}

private:
	Pair pairs[CLASS_MAP_INTERP_PAIRS_MAX_COUNT];
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
	static int intLInterp(long x, long x0, long y0, long x1, long y1)
	{
		if ((x1 - x0) == 0)
			return (int)( (y0 + y1 + 1) / 2 );
		return (int)( y0 + ((x - x0) * (y1 - y0) + (x1 - x0) / 2) / (x1 - x0) );
	}
};

class RobotArmState
{
public:
	enum NamedPosition { 
		None, 
		CenterSonar, 
		Center, 
		Rest, 
		Cup,
		MaxHeight, 
		MinHeight, 
		MaxRadius, 
		MinRadius,
		A, B, C, D
	};

	enum Direction { Vertical, Radial };

	struct MemberAngles	{ int boom1, boom2, turret;	};

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

	RobotArmState( 
		uint8_t pwrEnablePin,
		uint8_t pwrFeedbackPin,
		RobotArmMember& boom1,
		RobotArmMember& boom2,
		RobotArmMember& turret,
		RobotArmMember& claw,
		PositionVector preset[PRESET_POSITIONS_COUNT],
		Pair pairArray[BOOM2_MIN_PAIR_ARRAY_COUNT]
	);

	void init();

	//int getRadius();	// endeffector working radius
	//int getHeight();	// endeffector working height
	//int getTheta();		// turret angle
	EndEffectorPositionVector * getPositionVector();

	PositionVector memberAnglesToPosition(int b1, int b2, int turret);

	bool isServoPowerOn();
	void servoPowerOn();
	void servoPowerOff();

	void attach();
	void attachSafe();

	void goToPosition(int height, int radius, int theta);
	void goToPosition(PositionVector p);
	void goToPosition(NamedPosition namedPos);
	void sweep();
	void RobotArmState::sweepPresetPositions();
	void constrainedMove(Direction dir, int value);

	bool verifyPosition(PositionVector& p);

private:
	void determineExtents();

	EndEffectorPositionVector pos;
	PositionVector posCenterSonar, posCenter, posRest, posCup;
	PositionVector posMaxHeight, posMinHeight, posMaxRadius, posMinRadius;
	PositionVector posA, posB, posC, posD;
	const uint8_t pwrEnablePin, pwrFeedbackPin;
	RobotArmMember &boom1, &boom2, &turret, &claw;
	RobotArmMember* memberList[4];
	InterpolatedMap interpMapBoom2Min;
};

#endif // RobotArmState_h