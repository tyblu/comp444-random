#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

const int motor_pin = 9; // PWM driving NPN to control motor
const int vA_pin = A0;  // to motor +ve
const int vB_pin = A1;  // to motor -ve and shunt top
const int vC_pin = A2;  // to shunt bottom and collector

// Define various ADC prescaler (https://goo.gl/qLdu2e)
const unsigned char prescale_002 = (1 << ADPS0);
const unsigned char prescale_004 = (1 << ADPS1);
const unsigned char prescale_008 = (1 << ADPS1) | (1 << ADPS0);
const unsigned char prescale_016 = (1 << ADPS2);
const unsigned char prescale_032 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char prescale_064 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char prescale_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

void setup() {
  pinMode( motor_pin, OUTPUT );
  
  ADCSRA &= ~prescale_128;  // remove bits set by Arduino library
  ADCSRA |= prescale_016;    // set our own prescaler

  lcd.begin(16, 2); //Initialize the 16x2 LCD
  lcd.clear();
  lcd.print("Testing 16MHz/16");
  lcd.setCursor(0,1);
  lcd.print("v1.01-20170430");
  delay(1500);
  lcd.clear();
}

void loop() {

  unsigned long t0 = 0;
  unsigned long t[100] = {0};
  unsigned int t_index = 0;

  unsigned int while_loop_counter = 0;

  int node_a, node_b, node_c;
  int node_a_temp, node_b_temp, node_c_temp;

  analogWrite( motor_pin, 153 );

  while ( true ) {

    t[t_index++] = micros();

    node_a = analogRead( vA_pin );
    t[t_index++] = micros();
    node_a = analogRead( vA_pin );
    t[t_index++] = micros();

    node_b = analogRead( vA_pin );
    t[t_index++] = micros();
    node_b = analogRead( vA_pin );
    t[t_index++] = micros();
    
    node_c = analogRead( vA_pin );
    t[t_index++] = micros();
    node_c = analogRead( vA_pin );
    t[t_index++] = micros();

    lcd.clear();
    lcd.print( "vA: " );
    lcd.print( vA_pin ); lcd.setCursor(11,0); lcd.print(while_loop_counter);
    lcd.setCursor( 0,1 );
    lcd.print( " t1: " );
    lcd.print( t[t_index-6] - t[t_index-7] );
    lcd.print( " t2: " );
    lcd.print( t[t_index-5] - t[t_index-6] );
    delay(1000);

    lcd.clear();
    lcd.print( "vB: " );
    lcd.print( vA_pin ); lcd.setCursor(11,0); lcd.print(while_loop_counter);
    lcd.setCursor( 0,1 );
    lcd.print( " t1: " );
    lcd.print( t[t_index-4] - t[t_index-5] );
    lcd.print( " t2: " );
    lcd.print( t[t_index-3] - t[t_index-4] );
    delay(1000);

    lcd.clear();
    lcd.print( "vC: " );
    lcd.print( vA_pin ); lcd.setCursor(11,0); lcd.print(while_loop_counter);
    lcd.setCursor( 0,1 );
    lcd.print( " t1: " );
    lcd.print( t[t_index-2] - t[t_index-3] );
    lcd.print( " t2: " );
    lcd.print( t[t_index-1] - t[t_index-2] );
    delay(1000);

    if ( t_index > (100-1-6) ) { t_index = 0; }
    if ( ++while_loop_counter > 999 ) { while_loop_counter = 0; }
    
  }

}