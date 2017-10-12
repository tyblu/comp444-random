/**
 *             pid_example.ino
 * Purpose:    Demonstrates PID control of end-effector height.
 *
 * @author:    Tyler Lucas
 * Student ID: 3305203
 * Date:       October 11, 2017
 * Version     0.1
 * 
 * References: PID Without a PhD, by Tim Wescott, Embedded Sys. Prog. - Oct 2000
********************************************************************************
 */

#include <Servo.h>

#define boomArmPin 11   // max 110, min 50'ish
#define boomArmSensorPin A0

#define angleMin 30
#define angleMax 140
#define angleInitial 90
#define angleAdjustmentMagnitude 1

#define usingNoPID true
#define usingPOnly false
#define usingIOnly false
#define usingPI false
#define usingPID false

static inline int8_t sgn(int val)
{
  if (val < 0) return -1;
  if (val == 0 ) return 0;
  return 1;
}

Servo boomArmServo;
int boomArmAngle;
int boomArmSensorValue;

const int boomArmLimits[2] = { 70, 115 };

// angle = slope * sensor value + offset
const double boomArmSensorSlope = 1.1131363501;
const double boomArmSensorOffset = -147.1303003827;

void setup()
{
  boomArmServo.attach( boomArmPin );

  Serial.begin(9600);

  boomArmSensorValue = analogReadAverage( boomArmSensorPin, 100 );
  boomArmAngle = boomArmSensorSlope * boomArmSensorValue + boomArmSensorOffset;
  boomArmAngle = constrain(boomArmAngle, boomArmLimits[0], boomArmLimits[1]);
  boomArmServo.write(boomArmAngle);

  Serial.println("angle = slope * sensor value + offset");
  Serial.print( boomArmSensorSlope );
  Serial.print(" * ");
  Serial.print( boomArmSensorValue );
  Serial.print(" + ");
  Serial.print( boomArmSensorOffset );
  Serial.print(" = ");
  Serial.print( boomArmAngle );
  Serial.println();
}

void loop()
{
  int target = boomArmAngle + 15 * sin( millis() );
  target = constrain(target, boomArmLimits[0], boomArmLimits[1]);
  Serial.println(target); delay(100);
  int difference = target - boomArmAngle;
  
  while ( boomArmAngle != target )
  {
    difference = target - boomArmAngle;
    boomArmAngle += sgn(difference) * angleAdjustmentMagnitude;
    boomArmAngle = constrain(boomArmAngle, boomArmLimits[0], boomArmLimits[1]);
    boomArmServo.write(boomArmAngle);
    delay(25);

    Serial.print(boomArmAngle);
    Serial.write(',');
    Serial.print(difference);
    Serial.write(',');
    Serial.println(sgn(difference));
  }
}

int analogReadAverage(int pin, int count)
{
  long sum = 0;
  for (int i=0; i<count; i++)
    sum += analogRead( pin );
  return sum / count;
}
