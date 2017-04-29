/********************************************************************
 * Based on SparkFun Inventor's Kit example #12: SPINNING A MOTOR,
 * example #7: TEMPERATURE SENSOR, and example #15: LIQUID CRYSTAL 
 * DISPLAY (LCD)
 * 
 * Set motor speed from serial monitor. Put motor driving transistor
 * in high impedance (motorPin set to LOW) for a short time, 
 * measure the back-emf during this pause, then continue driving the
 * motor. Measured value, EMF voltage and motor speed are output to 
 * an LCD.
 *******************************************************************/

#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

const int motor_pin = 9; // PWM driving NPN to control motor
const int emf_pin = A0;  // back-emf related input
const int pause_pin = 8; // digital driving NPN to pause motor

void setup() {
  pinMode( motor_pin, OUTPUT );
  pinMode( pause_pin, OUTPUT );

  lcd.begin(16, 2); //Initialize the 16x2 LCD
  lcd.clear();  // clear any old data displayed on the LCD
  lcd.print("Hello, COMP444!");
  lcd.setCursor(0,1);
  lcd.print("emf to RPM v2.1");
  delay(1500);
}

void loop() {

  unsigned long t_pause = 500;  // pause time in [us], ulong for quick add to micros()
  unsigned long t_limit;

  int motor_speed_target_last = 0;
  int motor_speed_target = 0;
  int motor_speed_current = 0;
  int motor_speed_steps = 0;
  int motor_speed_next = 0;
  int motor_speed_target_to_current = 0;
  int motor_speed_target_to_current_abs = 0;

  int adc_values[100] = {0};
  
  while ( true ) {

    digitalWrite( pause_pin, HIGH );  // connect to common

    motor_speed_target = map(; // 1V-3V rating/5V supply=20-60% duty=51-153pwm
    while ( motor_speed_target != motor_speed_current )
    {   // ramp motor speed to target
      
      motor_speed_target_to_current = motor_speed_target - motor_speed_current;
      motor_speed_target_to_current_abs = abs( motor_speed_target_to_current );
//      motor_speed_steps = ( motor_speed_target_to_current_abs + 10 )/10; // most sig. digit+1
//      motor_speed_next = motor_speed_current + motor_speed_target_to_current / motor_speed_steps;
      motor_speed_next = motor_speed_current + motor_speed_target_to_current / motor_speed_target_to_current_abs; // +/-1 at a time
  
      analogWrite( motor_pin, motor_speed_next );
//      delay(200); // slow ramp
      delay(50);  // lower delay with only single increment changes
  
      motor_speed_current = motor_speed_next;
    }

//    // pause motor current, measure emf, reconnect driver
//    digitalWrite( pause_pin, LOW );   // turn off motor driver
//    delayMicroseconds( 250 );   // allow emf to stabilize
//    t_limit = micros() + t_pause; // set time limit for measurements
//    int n = 0;
//    while ( micros() < t_limit ) {
//      adc_values[n++] = analogRead( emf_pin ); // takes ~100us
//      if ( n > 98 ) { break; } // if this happens, there's a timer issue
//    }
//    digitalWrite( pause_pin, HIGH );  // turn on motor driver
  }
}
