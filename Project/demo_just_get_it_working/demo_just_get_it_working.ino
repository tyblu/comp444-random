/*
SparkFun Inventor's Kit
Example sketch 08-2

SINGLE SERVO

  Sweep a servo back and forth through its full range of motion.

  A "servo", short for servomotor, is a motor that includes 
  feedback circuitry that allows it to be commanded to move to
  specific positions. This one is very small, but larger servos
  are used extensively in robotics to control mechanical arms,
  hands, etc. You could use it to make a (tiny) robot arm,
  aircraft control surface, or anywhere something needs to be
  moved to specific positions.


This sketch was written by SparkFun Electronics,
with lots of help from the Arduino community.
This code is completely free for any use.
Visit http://learn.sparkfun.com/products/2 for SIK information.
Visit http://www.arduino.cc to learn about the Arduino.

Version 2.0 6/2012 MDG
*/


#include <Servo.h>  // servo library

Servo servo[4];  // servo control objects
int angles[4], speeds[4];

void setup()
{
  servo[1].attach( 9 );
  servo[2].attach( 6 );
  servo[3].attach( 5 );
  servo[4].attach( 3 );

  int n;
  for ( n=0; n<4; n++ )
  {
    servo[n].write( 90 );
  }
  
  Serial.begin(9600);
}


void loop()
{
  int n;
  for ( n=0; n<4; n++ )
  {
    Serial.print("\nEnter new angle for servo ");
    Serial.write('#');
    Serial.print( n );
    Serial.print(": ");

    while ( Serial.available() == 0 ) { }
    
    angles[n] = Serial.parseInt();
    Serial.print( angles[n] );
    Serial.println(" deg entered.");
    
    angles[n] = constrain( angles[n], 0, 180 );

    Serial.print("Setting angle to ");
    Serial.print( angles[n] );
    Serial.println(" deg.");

    servo[n].write( angles[n] );
  }
}
