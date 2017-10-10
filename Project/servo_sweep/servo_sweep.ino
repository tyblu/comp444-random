/**
 *             servo_sweep.ino
 * Purpose:    Used to calibrate measured position vs input angle.
 *
 * @author:    Tyler Lucas
 * Student ID: 3305203
 * Date:       October 9, 2017
 * Version     1.1
 * 
 * References: N/A
 * 
 */


#include <Servo.h>

#define boomArmPin 11
#define mainArmPin 10
#define clawPin 9       // max 140, min 69
#define swivelPin 6     // max 180, min 10

#define angleMin 60
#define angleMax 110
#define angleInitial 90
#define angleAdjustmentMagnitude 2

Servo servo;
int angle = angleInitial;
int angleAdjustment = angleAdjustmentMagnitude;

void setup()
{
  servo.attach( clawPin );

  Serial.begin(9600);
}

void loop()
{
  while ( angle >= angleMin && angle <= angleMax )
  {
    servo.write( angle );
    Serial.println( angle );
    angle += angleAdjustment;
    delay(50);
  }

  angleAdjustment *= -1;  // reverse
  angle += 2 * angleAdjustment;
}
