#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

const int motor_pin = 9; // PWM driving NPN to control motor
const int vA_pin = A0;  // to motor +ve
const int vB_pin = A1;  // to motor -ve and shunt top
const int vC_pin = A2;  // to shunt bottom and collector

// Define various ADC prescaler (https://goo.gl/qLdu2e)
const unsigned char prescale_002 = (1 << ADPS0);                                //   8 MHz (615.4 kHz) ~   1.6 us
const unsigned char prescale_004 = (1 << ADPS1);                                //   4 MHz (307.7 kHz) ~   3.3 us
const unsigned char prescale_008 = (1 << ADPS1) | (1 << ADPS0);                 //   2 MHz (153.8 kHz) ~   6.5 us
const unsigned char prescale_016 = (1 << ADPS2);                                //   1 MHz ( 76.9 kHz) ~  13 us   This one looks like the winner.
const unsigned char prescale_032 = (1 << ADPS2) | (1 << ADPS0);                 // 500 kHz ( 38.5 kHz) ~  26 us
const unsigned char prescale_064 = (1 << ADPS2) | (1 << ADPS1);                 // 250 kHz ( 19.2 kHz) ~  52 us
const unsigned char prescale_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // 125 kHz ( 9.6 kHz) ~  104 us

#define NODE_VOLTAGES_ARRAY_SIZE 100

struct node_voltages {
  unsigned int vdata[NODE_VOLTAGES_ARRAY_SIZE];
  unsigned int vmin = 1023;   // default initial value
  unsigned int vmax = 0;      // default initial value
  unsigned int vavg;
};

const unsigned int lcd_delay = 3000;

void setup() {
  pinMode( motor_pin, OUTPUT );
  
  ADCSRA &= ~prescale_128;    // remove bits set by Arduino library
  ADCSRA |= prescale_016;     // set our own prescaler

  lcd.begin(16, 2); //Initialize the 16x2 LCD
  lcd.clear();
  lcd.print("Testing 16MHz/16");
  lcd.setCursor(0,1);
  lcd.print("v1.06-20170501");
  delay(1500);
  lcd.clear();
}

void loop() {

  node_voltages a, b, c;
  int n;
  
  unsigned long t[100] = {0};
  unsigned int t_index = 0;
  unsigned long t_offset = get_timestamp_offset();
  unsigned long t_diffs[100] = {0};
  unsigned int t_diffs_index = 0;

  unsigned int while_loop_counter = 0;

  analogWrite( motor_pin, 153 );
  delay(1000);                    // allow electronics to stabilize

  while ( true ) {

    t[t_index++] = micros();
    for ( n=0; n<NODE_VOLTAGES_ARRAY_SIZE; n++) { a.vdata[n] = analogRead( vA_pin ); }
    t[t_index++] = micros();
    t_diffs[t_diffs_index++] = t[t_index-1] - t[t_index-2] - t_offset;
    
    t[t_index++] = micros();
    for ( n=0; n<NODE_VOLTAGES_ARRAY_SIZE; n++) { b.vdata[n] = analogRead( vB_pin ); }
    t[t_index++] = micros();
    t_diffs[t_diffs_index++] = t[t_index-1] - t[t_index-2] - t_offset;
    
    t[t_index++] = micros();
    for ( n=0; n<NODE_VOLTAGES_ARRAY_SIZE; n++) { c.vdata[n] = analogRead( vC_pin ); }
    t[t_index++] = micros();
    t_diffs[t_diffs_index++] = t[t_index-1] - t[t_index-2] - t_offset;

    compute_node_voltages_vavg_vmin_vmax( a );
    compute_node_voltages_vavg_vmin_vmax( b );
    compute_node_voltages_vavg_vmin_vmax( c );

    lcd.clear();
    lcd.print( "vAavg: " );
    lcd.print( a.vavg ); lcd.setCursor(12,0); lcd.print(while_loop_counter);
    lcd.setCursor( 0,1 );
    lcd.print( " t: " );
    lcd.print( t_diffs[ t_diffs_index - 3 ] );
    lcd.print(" (");
    lcd.print( t_diffs[ t_diffs_index - 3 ]/NODE_VOLTAGES_ARRAY_SIZE );
    lcd.print("per)");
    delay(lcd_delay);
    lcd.setCursor( 0,0 ); lcd.print( "vAmax: " ); lcd.print( a.vmax );
    delay(lcd_delay);
    lcd.setCursor( 0,0 ); lcd.print( "vAmin: " ); lcd.print( a.vmin );
    delay(lcd_delay);

    lcd.clear();
    lcd.print( "vBavg: " );
    lcd.print( b.vavg ); lcd.setCursor(12,0); lcd.print(while_loop_counter);
    lcd.setCursor( 0,1 );
    lcd.print( " t: " );
    lcd.print( t_diffs[ t_diffs_index - 2 ] );
    lcd.print(" (");
    lcd.print( t_diffs[ t_diffs_index - 2 ]/NODE_VOLTAGES_ARRAY_SIZE );
    lcd.print("per)");
    delay(lcd_delay);
    lcd.setCursor( 0,0 ); lcd.print( "vBmax: " ); lcd.print( b.vmax );
    delay(lcd_delay);
    lcd.setCursor( 0,0 ); lcd.print( "vBmin: " ); lcd.print( b.vmin );
    delay(lcd_delay);

    lcd.clear();
    lcd.print( "vCavg: " );
    lcd.print( c.vavg ); lcd.setCursor(12,0); lcd.print(while_loop_counter);
    lcd.setCursor( 0,1 );
    lcd.print( " t: " );
    lcd.print( t_diffs[ t_diffs_index - 1 ] );
    lcd.print(" (");
    lcd.print( t_diffs[ t_diffs_index - 1 ]/NODE_VOLTAGES_ARRAY_SIZE );
    lcd.print("per)");
    delay(lcd_delay);
    lcd.setCursor( 0,0 ); lcd.print( "vCmax: " ); lcd.print( c.vmax );
    delay(lcd_delay);
    lcd.setCursor( 0,0 ); lcd.print( "vCmin: " ); lcd.print( c.vmin );
    delay(lcd_delay);

    if ( t_index > (100-1-5) ) { t_index = 0; t_diffs_index = 0; }
    if ( ++while_loop_counter > 999 ) { while_loop_counter = 0; lcd.clear(); lcd.print("wtf is going on?"); }
    
  }

}


// attempt to figure out the overhead time for the adc reading loops, etc.
unsigned long get_timestamp_offset() {
  unsigned long timestamp[2] = {0};
  unsigned int t_index = 0;
  int n;

  timestamp[t_index++] = micros();

  for ( n=0; n<NODE_VOLTAGES_ARRAY_SIZE; n++ ) { }

  timestamp[t_index++] = micros();

  return( timestamp[1] - timestamp[0] );
}

// update avg, max, and min values
void compute_node_voltages_vavg_vmin_vmax( node_voltages& node_voltages_struct ) {
  
  unsigned long node_sum = 0;
  
  int n;
  for ( n=0; n<NODE_VOLTAGES_ARRAY_SIZE; n++ ){
    node_sum += node_voltages_struct.vdata[n];

    if ( node_voltages_struct.vmax < node_voltages_struct.vdata[n] ) {
      node_voltages_struct.vmax = node_voltages_struct.vdata[n];
    }

    if ( node_voltages_struct.vmin > node_voltages_struct.vdata[n] ) {
      node_voltages_struct.vmin = node_voltages_struct.vdata[n];
    }
  }
  node_voltages_struct.vavg = node_sum / NODE_VOLTAGES_ARRAY_SIZE;
}












