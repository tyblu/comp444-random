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
int angles[4] = { 90, 65, 90, 60 };   // initial angles
int constraints[4][2] = { {0, 180}, {65, 125}, {65, 140}, {60, 150} };

void setup()
{
  servo[0].attach( 9 );
  servo[1].attach( 6 );
  servo[2].attach( 5 );
  servo[3].attach( 3 );
  for ( int n=0; n<4; n++ )
    servo[n].write( angles[n] );
  
  Serial.begin(9600);
}

void loop()
{
  Serial.println();
  
  Serial.print("Servo #");
  while ( Serial.available() == 0 ) { }
  int servo_num = Serial.parseInt();
  Serial.println( servo_num );

  Serial.print("Angle: ");
  while ( Serial.available() == 0 ) { }
  int angle = Serial.parseInt();
  Serial.println( angle );

  angle = constrain(angle, constraints[servo_num][0], constraints[servo_num][1]);

  Serial.print("Setting servo #");
  Serial.print( servo_num );
  Serial.print(" to ");
  Serial.print( angle );
  Serial.println(" degrees.");

  int min_delay_time = 10, max_delay_time = 100, delay_time = max_delay_time;
  int dir = (angle - angles[servo_num]) / abs(angle - angles[servo_num]);
  while ( angles[servo_num] != angle )
  {
    angles[servo_num] += dir;
    servo[servo_num].write( angles[servo_num] );
    delay(delay_time);
    Serial.write('.');
    if ( angles[servo_num] % 15 == 0 )
      Serial.print(angles[servo_num]);

    if ( abs(angle - angles[servo_num]) > 15 && delay_time > min_delay_time )
      delay_time -= min_delay_time;
    else if ( abs(angle - angles[servo_num]) <= 15 && delay_time < max_delay_time)
      delay_time += min_delay_time;
  }
}
