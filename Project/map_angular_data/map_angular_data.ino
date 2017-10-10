/**
 *            map_angular_data.ino
 * Purpose:   Measures sensor values for servo angles. Used to
 *            calibrate each servo.
 *
 * @author:    Tyler Lucas
 * Student ID: 3305203
 * Date:       October 9, 2017
 * Version     1.0
 * 
 * References: N/A
 * 
 */


#include <Servo.h>  // servo library

#define servo0PWMPin 11
#define servo0sensorPin A0

#define angleMax 110
#define angleMin 30

Servo servo;
int angle = angleMin;
int angleAdjustment = 1;

void setup()
{
  servo.attach( servo0PWMPin );
  
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
      Serial.println( analogRead(servo0sensorPin) );
    }
    
    angle += angleAdjustment;
  }

  angleAdjustment *= -1;    // reverse direction
}

double analogReadAverage(int pin, unsigned int points)
{
  double sum = 0;
  for (unsigned int i=0; i<points; i++)
    sum += analogRead(pin);
  return sum / points;
}

