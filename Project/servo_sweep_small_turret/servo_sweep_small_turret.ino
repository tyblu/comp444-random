#include <Servo.h>

#define PRINTVAR(x,y) Serial.print(x); Serial.print(y)  // terminate with ';'

#define TURRET_SERVO_PWM_PIN 3
#define ANGLE_MINUTES_MAX 60 * 180
#define ANGLE_MINUTES_MIN 60 * 0

int16_t minToDeg(int16_t minutes);

uint32_t microsPerAngularMinute = 6 * 1000000 / ( 60 * 180 );  // 180deg/6sec, ~555us/min
const int16_t SERVO_ANGLE_NEAR_APOGEE = 10;  // slow down when this many deg from target
const uint32_t SERVO_ANGLE_APOGEE_PAUSE_TIME_MS = 3000;

Servo servo;

int16_t angle = minToDeg( (int16_t)(ANGLE_MINUTES_MIN) );
int16_t angleMinutes = (int16_t)( angle * 60 + 1 );
int16_t angleTarget = minToDeg( (int16_t)(ANGLE_MINUTES_MAX) );

int16_t dir = 1; // 1 increasing, -1 decreasing

uint32_t nextMinuteIncrementTimeMicros;

void setup() {
  Serial.begin(9600);
  
  servo.attach(TURRET_SERVO_PWM_PIN);
  servo.write(angle);
  
  delay(1000);
}

void loop() {
  servo.write(angle);
  
  nextMinuteIncrementTimeMicros = micros() + microsPerAngularMinute;
  if ( abs( angle - angleTarget ) < SERVO_ANGLE_NEAR_APOGEE )
    nextMinuteIncrementTimeMicros += 1*microsPerAngularMinute;  // 1/(1+1) speed
  
  int32_t diff;
  do {
    diff = micros() - nextMinuteIncrementTimeMicros; // take a nop
  } while( diff < 0 );

  if ( diff > 20 )
  {
    Serial.print("Rate too high. Lowering it to ");
    microsPerAngularMinute += microsPerAngularMinute / 10;
    Serial.print( microsPerAngularMinute );
    Serial.println(" microseconds per angular minute (1/60th of a degree).");
  }

  if ( angleMinutes <= ANGLE_MINUTES_MIN || angleMinutes >= ANGLE_MINUTES_MAX )
  {
    dir *= -1;
    delay(SERVO_ANGLE_APOGEE_PAUSE_TIME_MS);
  }
  
  angleMinutes += dir;

  angle = minToDeg( angleMinutes );

  if (angleMinutes > 0 && angleMinutes % (15 * 60) == 0)
  {
    PRINTVAR("angle=", angle);
    PRINTVAR(", angleMinutes=", angleMinutes);
    PRINTVAR(", dir=", dir);
    Serial.println();
  }
}

int16_t minToDeg(int16_t minutes)
{
  return (int16_t)(( minutes + 60 / 2 )/60);
}
