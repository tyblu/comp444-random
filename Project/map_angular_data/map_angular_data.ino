/**
 *            map_angular_data.ino
 * Purpose:   Measures sensor values for servo angles. Used to
 *            calibrate each servo.
 *
 * @author:    Tyler Lucas
 * Student ID: 3305203
 * Date:       October 9, 2017
 * Version     1.1
 * 
 * References: N/A
 * 
 */


#include <Servo.h>  // servo library

#define boomArmPin 11   // max 110, min ?
#define mainArmPin 10   // max ?, min ?
#define clawPin 9       // max 140, min 69
#define swivelPin 6     // max 180, min 10

#define boomArmSensorPin A0
#define mainArmSensorPin A1
#define clawSensorPin A2
#define swivelSensorPin A3

#define angleMin 69
#define angleMax 140
#define angleInitial 90
#define angleAdjustmentMagnitude 1

Servo servo;
int angle = angleMin;
int angleAdjustment = angleAdjustmentMagnitude;

void setup()
{
  servo.attach( clawPin );
  
  Serial.begin(9600);

  delay(1000);
}

void loop()
{
  while ( angle <= angleMax && angle >= angleMin)
  {
    servo.write( angle );
    delay(50);    // let arm settle

    // print
    for (int i=0; i<100; i++)
    {
      Serial.print( angle );
      Serial.write(',');
      Serial.println( analogRead( clawSensorPin ) );
    }
    
    angle += angleAdjustment;
  }

  angleAdjustment *= -1;    // reverse direction
}
