/**
 *             demo_just_get_it_working
 *             demo_just_get_it_working.ino
 * Purpose:    Quick and dirty scripts to test parts of my project robot.
 *
 * @author:    Tyler Lucas
 * Student ID: 3305203
 * Date:       May 5, 2017
 * Version     1.1
 * 
 * References: SparkFun Inventor's Kit Example sketch 08-2: SINGLE SERVO Version 2.0 6/2012 MDG
 * 
 */


#include <Servo.h>  // servo library

Servo servo[4];  // servo control objects
int angles[4] = { 90, 65, 90, 60 };   // initial angles

void setup()
{
  servo[0].attach( 9 );   // swivel
  servo[1].attach( 6 );   // boom
  servo[2].attach( 5 );   // pincher
  servo[3].attach( 3 );   // main arm
  for ( int n=0; n<4; n++ )
    servo[n].write( angles[n] );
  
  Serial.begin(9600);
}

void loop()
{
  Serial.println();
  
  Serial.print("Servo #");
  int servo_num = -1;
  while ( servo_num < 0 || servo_num > 3 )
  {
    while ( Serial.available() == 0 ) { }
    servo_num = Serial.parseInt();
    if ( servo_num < 0 || servo_num > 3 )
      Serial.print("\n Umm.. nope. Try again.\nServo #");
  }
  Serial.println( servo_num );

  Serial.print("Angle: ");
  while ( Serial.available() == 0 ) { }
  int angle = Serial.parseInt();
  Serial.println( angle );

  angle = constrain_servo_angle(servo_num, angle);

  Serial.print("Setting servo #");
  Serial.print( servo_num );
  Serial.print(" to ");
  Serial.print( angle );
  Serial.println(" degrees.");

  go_to(servo_num, angle);

  Serial.print("Servo #");
  Serial.print( servo_num );
  Serial.print(" set to ");
  Serial.print( angles[servo_num] );
  Serial.println(" degrees.");
}

int constrain_servo_angle(int servo_number, int input_angle)
{
  switch ( servo_number )
  {
  case 0: return constrain(input_angle, 0, 180);
  case 1:
    if (angles[3] < 60)
      return constrain(input_angle, formulaZ(angles[3]), formulaA(angles[3]));
    else if (angles[3] >= 60 && angles[3] < 70)
      return constrain(input_angle, formulaY(angles[3]), formulaB(angles[3]));
    else if (angles[3] >= 70 && angles[3] < 80)
      return constrain(input_angle, formulaY(angles[3]), formulaC(angles[3]));
    else if (angles[3] >= 80 && angles[3] < 100)
      return constrain(input_angle, formulaX(angles[3]), formulaC(angles[3]));
    else if (angles[3] >= 100 && angles[3] < 110)
      return constrain(input_angle, formulaX(angles[3]), formulaD(angles[3]));
    else if (angles[3] >= 110 && angles[3] < 120)
      return constrain(input_angle, formulaW(angles[3]), formulaE(angles[3]));
    else if (angles[3] >= 120 && angles[3] < 140)
      return constrain(input_angle, formulaV(angles[3]), formulaE(angles[3]));
    else if (angles[3] >= 140)
      return constrain(input_angle, formulaU(angles[3]), formulaE(angles[3]));
    else
      return -1;
  case 2: return constrain(input_angle, 65, 140);
  case 3: return constrain(input_angle, 60, 140);
  default: return -1;
  }
}

int formulaA(int angle) { return 140; }
int formulaB(int angle) { return 140 + (angle - 60) * (135-140)/(70-60); }
int formulaC(int angle) { return 135; }
int formulaD(int angle) { return 135 + (angle - 100) * (130-135)/(110-100); }
int formulaE(int angle) { return 130; }
int formulaZ(int angle) { return 65; }
int formulaY(int angle) { return 65 + (angle - 60) * (54-65)/(80-60); }
int formulaX(int angle) { return 54 + (angle - 80) * (20-54)/(110-80); }
int formulaW(int angle) { return 20 + (angle - 110) * (25-20)/(120-110); }
int formulaV(int angle) { return 25 + (angle - 120) * (18-25)/(140-120); }
int formulaU(int angle) { return 18; }

int constrain_servo_angle_conservatively(int servo_number, int input_angle)
{
  switch ( servo_number )
  {
  case 0: return constrain(input_angle, 15, 165);
  case 1: return constrain(input_angle, 65, 130);
  case 2: return constrain(input_angle, 65, 140);
  case 3: return constrain(input_angle, 60, 140);
  default: return -1;
  }
}

/*
 * Increments towards the intended angle, keeping other angles in valid ranges.
 */
void go_towards(int servo_number, int angle)
{
  angle = constrain_servo_angle(servo_number, angle);
  int diff = angle - angles[servo_number];
  
  int dir;
  if (diff == 0)
    dir = 0;
  else
    dir = diff / abs(diff);
  
  angles[servo_number] += dir;
  servo[servo_number].write( angles[servo_number] );
  
//  if ( angles[servo_number] % 15 == 0 )
//    Serial.print(angles[servo_number]);
}

void go_to(int servo_number, int angle)
{
  int min_delay_time = 10, max_delay_time = 100, delay_time = max_delay_time;
  
  int previous_angle;
  do {
    previous_angle = angles[servo_number];
    
    go_towards(servo_number, angle);
    
    delay(delay_time);

    if ( abs(angle - angles[servo_number]) > 15 && delay_time > min_delay_time )
      delay_time -= min_delay_time;
    else if ( abs(angle - angles[servo_number]) <= 15 && delay_time < max_delay_time)
      delay_time += min_delay_time;
  } while ( abs(previous_angle - angles[servo_number]) != 0 );
}

