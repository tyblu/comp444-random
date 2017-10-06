/**
 *             robot_arm_assign2_extra.ino
 * Purpose:    Keeps the sonar sensor at a constant distance from a surface.
 *
 * @author:    Tyler Lucas
 * Student ID: 3305203
 * Date:       October 3, 2017
 * Version     1.0
 * 
 * References: N/A
 * 
 */


#include <Servo.h>  // servo library

#define sonarTrigPin 13
#define sonarEchoPin 12
#define servo0PWMPin 11

Servo servo;
int angle = 90;

void setup()
{
  servo.attach( servo0PWMPin );

  pinMode(sonarTrigPin, OUTPUT);
  pinMode(sonarEchoPin, INPUT);
  
  Serial.begin(9600);

  servo.write( angle );
  delay(1000);
}

void loop()
{
  double d_now = readSonarAverage(10);
  double d_hold = d_now;
  double d_error;

  while (1)
  {
    d_error = d_now - d_hold;

    if (d_error > 5)
      angle -= 2;

    if (d_error < -5)
      angle += 2;

    angle = constrain(angle, 30, 120);

    servo.write( angle );

    Serial.println();
    Serial.print("Distance (current): ");
    Serial.print( d_now );
    Serial.print(" [mm] -- (holding): ");
    Serial.print( d_hold );
    Serial.print(" [mm] - Angle: ");
    Serial.print( angle );
    Serial.print("[deg]");

    delay(25);

    d_now = readSonarAverage(5);
  }
}

double readSonar()
{
  long duration;

  digitalWrite(sonarTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(sonarTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sonarTrigPin, LOW);

  duration = pulseIn(sonarEchoPin, HIGH);  
  return 0.146 * duration - 23.843;  // returns in mm
}

double readSonarAverage(int count)
{
  double sum = 0;
  for (int i=0; i<count; i++)
    sum += readSonar();
  return sum / count;
}

double analogReadAverage(int pin, int count)
{
  double sum = 0;
  for (int i=0; i<count; i++)
    sum += analogRead(pin);
  return sum / count;
}

