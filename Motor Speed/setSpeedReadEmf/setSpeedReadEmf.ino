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

const int motor_pin = 9; // conected to transistor base to drive motor
const int emf_pin = A0;  // analog input related to motor back-emf

void setup() {
  pinMode(motor_pin, OUTPUT);  // set up the pin as an OUTPUT

  lcd.begin(16, 2); //Initialize the 16x2 LCD
  lcd.clear();  // clear any old data displayed on the LCD
  lcd.print("Hello, COMP444!");
  lcd.setCursor(0,1);
  lcd.print("emf to RPM v2.0");
  delay(1500);
}

void loop() {

  int t_pause = 500;    // pause time in microseconds
  unsigned long t_now, t_then;

  int motor_speed_target_last = 0;
  int motor_speed_target = 0;
  int motor_speed_current = 0;
  int motor_speed_steps = 0;
  int motor_speed_next = 0;
  int motor_speed_target_to_current = 0;
  int motor_speed_target_to_current_abs = 0;

  int adc_current = 0;
  int adc_history[100] = {0};
  
  while ( true ) {

    motor_speed_target = 181;
    while ( motor_speed_target != motor_speed_current )
    {   // ramp motor speed to target
      
      motor_speed_target_to_current = motor_speed_target - motor_speed_current;
      motor_speed_target_to_current_abs = abs( motor_speed_target_to_current );
      motor_speed_steps = ( motor_speed_target_to_current_abs + 10 )/10; // most sig. digit+1
      motor_speed_next = motor_speed_current + motor_speed_target_to_current / motor_speed_steps;
  
      analogWrite( motor_pin, motor_speed_next );
      delay(100);
  
      motor_speed_current = motor_speed_next;
    }
  
//    analogWrite( motor_pin, LOW );
//    delayMicroseconds( 100 );   // allow emf to stabilize
    adc_current = analogRead( emf_pin );
//    analogWrite( motor_pin, motor_speed_current );

    push_to_vector( adc_history, adc_current );
  }
}

void push_to_vector ( int (&vec)[100], int value ) {
  int n = 100-1;
  while ( n-- > 1 ){
    vec[n] = vec[n-1];  // shuffle down
  }
  vec[0] = value;       // push to top
}

