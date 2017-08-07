/**
 *             demo_just_get_it_working
 *             demo_just_get_it_working.ino
 * Purpose:    Quick and dirty scripts to test parts of my project robot.
 *
 * @author:    Tyler Lucas
 * Student ID: 3305203
 * Date:       May 5, 2017
 * Version     1.01
 * 
 * References: SparkFun Inventor's Kit Example sketch 08-2: SINGLE SERVO Version 2.0 6/2012 MDG
 * 
 */


#include <Servo.h>  // servo library

Servo servo[4];  // servo control objects
int angles[4], speeds[4];

void setup()
{
  servo[0].attach( 9 );
  servo[1].attach( 6 );
  servo[2].attach( 5 );
  servo[3].attach( 3 );

  int n;
  for ( n=0; n<4; n++ )
  {
    servo[n].write( 90 );
  }
  
  Serial.begin(9600);

  pinMode(13, OUTPUT);
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
