/**
 *             sonar_error_mapping.ino
 * Purpose:   Spits out sonar data while setting the boom arm servo at set angles
 *            in order to determine the precision and accuracy of the system. Can
 *            copy values into spreadsheet to extract information. May be used to 
 *            analyze step input response.
 *
 * @author:    Tyler Lucas
 * Student ID: 3305203
 * Date:       October 6, 2017
 * Version     1.3
 * 
 * References: N/A
 * 
 */


#include <Servo.h>  // servo library

#define sonarTrigPin 13
#define sonarEchoPin 12
#define servo0PWMPin 11

#define angleMax 40
#define angleMin 30
#define angleAdjustmentMagnitude 10

#define timeDelayToNextAngle 100

Servo servo;
int angle = angleMin;
int angleAdjustment = angleAdjustmentMagnitude;

void setup()
{
  servo.attach( servo0PWMPin );

  pinMode(sonarTrigPin, OUTPUT);
  pinMode(sonarEchoPin, INPUT);
  
  Serial.begin(9600);

  servo.write( angle );
  
  Serial.println();
  Serial.println("time [ms], input angle, sonar");

  delay(1000);
}

void loop()
{
  unsigned long timeStart = millis();
  unsigned long timestamp = timeStart;
  unsigned long timeNextAngle = timestamp + timeDelayToNextAngle;
  long sonarDuration;

  servo.write( angle );

  while ( timestamp < timeNextAngle )
  {
    timestamp = millis();
    sonarDuration = getSonarDuration();
    
    Serial.println();
    Serial.print( timestamp - timeStart );
    Serial.write(',');
    Serial.print( angle );
    Serial.write(',');
    Serial.print( sonarDuration );
  }

  angle += angleAdjustment;

  if ( angle > angleMax || angle < angleMin )
  {
    angleAdjustment *= -1;    // reverse direction
    angle += 2 * angleAdjustment;
  }
}

// Returns the duration of the sonar echo in microseconds.
long getSonarDuration()
{
  digitalWrite(sonarTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(sonarTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sonarTrigPin, LOW);

  return pulseIn(sonarEchoPin, HIGH);
}
