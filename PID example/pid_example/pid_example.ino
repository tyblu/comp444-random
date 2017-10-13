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
  Serial.begin(9600);

  boomArmSensorValue = analogReadAverage( boomArmSensorPin, 100 );
  boomArmAngle = boomArmSensorSlope * boomArmSensorValue + boomArmSensorOffset;
  while ( boomArmAngle < boomArmLimits[0] || boomArmAngle > boomArmLimits[1] )
  {
    boomArmSensorValue = analogReadAverage( boomArmSensorPin, 100 );
    boomArmAngle = boomArmSensorSlope * boomArmSensorValue + boomArmSensorOffset;
  }
//  boomArmAngle = constrain(boomArmAngle, boomArmLimits[0], boomArmLimits[1]);
  boomArmServo.write(boomArmAngle);
  boomArmServo.attach( boomArmPin );

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
//  if ( digitalRead(0) == HIGH )
//    target = ( boomArmLimits[0] + boomArmLimits[1] ) / 2;
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

class tybluServo
{
public:
  void setMinAngle(int);
  void setMaxAngle(int);
  void setServo(Servo);
  void setSensorPin(int);
  void setSensorSlope(double);
  void setSensorOffset(double);

  int getMinAngle();
  int getMaxAngle();
  int getAnalogAngle();
  int getLastAngle();

  void attach(int);
  void detach();
  void write(int);

  tybluServo(Servo);
  tybluServo(Servo, int, int);
  tybluServo(Servo, int, int, int, double, double);

private:
  int minAngle, maxAngle;
  Servo servo;
  int sensorPin;
  double sensorSlope, sensorOffset;

  unsigned int analogValues[1024] = {0};  // for mode tallying
};

tybluServo::tybluServo(Servo servo)
{
  tybluServo.servo = servo;
}

tybluServo::tybluServo(Servo servo, int minAngle, int maxAngle)
{
  tybluServo.servo = servo;
  tybluServo.minAngle = minAngle;
  tybluServo.maxAngle = maxAngle;
}

tybluServo::tybluServo(Servo servo, int minAngle, int maxAngle, 
  int sensorPin, double sensorSlope, double sensorOffset)
{
  tybluServo.servo = servo;
  tybluServo.minAngle = minAngle;
  tybluServo.maxAngle = maxAngle;
  tybluServo.sensorSlope = sensorSlope;
  tybluServo.sensorOffset = sensorOffset;
}

void tybluServo::setMinAngle(int minAngle)
{
  tybluServo.minAngle = minAngle;
}

void tybluServo::setMaxAngle(int maxAngle)
{
  tybluServo.maxAngle = maxAngle;
}

void tybluServo::setServo(Servo servo)
{
  tybluServo.servo = servo;
}

void tybluServo::setSensorPin(int sensorPin)
{
  tybluServo.sensorPin = sensorPin;
}

void tybluServo::setSensorSlope(double sensorSlope)
{
  tybluServo.sensorSlope = sensorSlope;
}

void tybluServo::setSensorOffset(double sensorOffset)
{
  tybluServo.sensorOffset = sensorOffset;
}

int tybluServo::getMinAngle()
{
  return tybluServo.minAngle;
}

int getMaxAngle()
{
  return tybluServo.maxAngle();
}

int getAnalogAngle()
{
  for (unsigned int i=0; i<1024; i++)
    tybluServo.analogValues[i] = 0;
  
  for (unsigned int i=0; i<200; i++)
    tybluServo.analogValues[ analogRead( tybluServo.sensorPin ) ] += 1;

  int maxCount = 0;
  int maxCountValue = 0;
  for (unsigned int i=0; i<1024; i++)
  {
    if ( tybluServo.analogValues[i] > maxCount )
    {
      maxCount = analogValues[i];
      maxCountValue = i;
    }
  }

  return maxCountValue;
}

int tybluServo::getLastAngle()
{
  return tybluServo.servo.read();
}

void tybluServo::attach(int pin)
{
  tybluServo.servo.attach( pin );
}

void tybluServo::detach()
{
  tybluServo.servo.detach();
}

void tybluServo::write(int angle)
{
  tybluServo.servo.write( angle );
}

