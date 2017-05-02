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

#define DATA_ARRAY_SIZE 1256
#define NODE_VOLTAGES_ARRAY_SIZE 10

struct node_voltages {
  unsigned int vmin[NODE_VOLTAGES_ARRAY_SIZE] = {0};
  unsigned int vmax[NODE_VOLTAGES_ARRAY_SIZE] = {0};
  unsigned int vavg[NODE_VOLTAGES_ARRAY_SIZE] = {0};
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
  lcd.print("v1.10-20170501");
  delay(1500);
  lcd.clear();
}

void loop() {

  int app = 1;
  lcd.clear(); lcd.print("Memory A:"); lcd.setCursor( 1,1 ); lcd.print( freeRam() ); delay(1000);

  node_voltages a, b, c;
  int m;
  
  lcd.clear(); lcd.print("Memory B:"); lcd.setCursor( 1,1 ); lcd.print( freeRam() ); delay(1000);
  
  unsigned int vdata[DATA_ARRAY_SIZE] = {0};
  int n;
  
  lcd.clear(); lcd.print("Memory C:"); lcd.setCursor( 1,1 ); lcd.print( freeRam() ); delay(1000);
  
  unsigned long t[6] = {0};
  unsigned long t_offset = get_timestamp_offset();
  unsigned long t_diffs[3] = {0};

  unsigned int while_loop_counter = 0;

  analogWrite( motor_pin, 153 );
  lcd.clear(); lcd.print("Just a moment"); lcd_slow_dots(3000, 3);  // allow electronics to stabilize

  lcd.clear(); lcd.print("Memory D:"); lcd.setCursor( 1,1 ); lcd.print( freeRam() ); delay(1000);

  while ( true ) {

    if ( n % 10 == 0 ) { lcd.clear(); lcd.print("Memory E"); lcd.print(n); lcd.print(":"); lcd.setCursor( 1,1 ); lcd.print( freeRam() ); delay(1000); }

    t[0] = micros();
    for ( m=0; m<NODE_VOLTAGES_ARRAY_SIZE; m++) {
      for ( n=0; n<DATA_ARRAY_SIZE; n++) { if ( n % 10 == 0 ) { lcd.clear(); lcd.print("m="); lcd.print(m); lcd.setCursor(1,7); lcd.print("n="); lcd.print(n); delay(250); }
        vdata[n] = analogRead( vA_pin );
      }
      compute_node_voltages_vavg_vmin_vmax( a, vdata, m);
    }
    t[1] = micros();
    t_diffs[0] = t[1] - t[0] - t_offset;

    t[2] = micros();
    for ( m=0; m<NODE_VOLTAGES_ARRAY_SIZE; m++) {
      for ( n=0; n<DATA_ARRAY_SIZE; n++) { if ( n % 10 == 0 ) { lcd.clear(); lcd.print("m="); lcd.print(m); lcd.setCursor(1,7); lcd.print("n="); lcd.print(n); delay(250); }
        vdata[n] = analogRead( vB_pin );
      }
      compute_node_voltages_vavg_vmin_vmax( b, vdata, m);
    }
    t[3] = micros();
    t_diffs[1] = t[3] - t[2] - t_offset;
    
    t[4] = micros();
    for ( m=0; m<NODE_VOLTAGES_ARRAY_SIZE; m++) {
      for ( n=0; n<DATA_ARRAY_SIZE; n++) { if ( n % 10 == 0 ) { lcd.clear(); lcd.print("m="); lcd.print(m); lcd.setCursor(1,7); lcd.print("n="); lcd.print(n); delay(250); }
        vdata[n] = analogRead( vC_pin );
      }
      compute_node_voltages_vavg_vmin_vmax( c, vdata, m);
    }
    t[5] = micros();
    t_diffs[2] = t[5] - t[4] - t_offset;
    
lcd.clear(); lcd.print("made it!"); delay(2000); // debugging

    lcd.clear();
    lcd.print( "vAavg: " );
    lcd.print( a.vavg[1] ); lcd.setCursor(12,0); lcd.print(while_loop_counter);
    lcd.setCursor( 0,1 );
    lcd.print( " t: " );
    lcd.print( t_diffs[0] );
    lcd.print(" (");
    lcd.print( t_diffs[0] /NODE_VOLTAGES_ARRAY_SIZE );
    lcd.print("per)");
    delay(lcd_delay);
    lcd.setCursor( 0,0 ); lcd.print( "vAmax: " ); lcd.print( a.vmax[1] );
    delay(lcd_delay);
    lcd.setCursor( 0,0 ); lcd.print( "vAmin: " ); lcd.print( a.vmin[1] );
    delay(lcd_delay);

    lcd.clear();
    lcd.print( "vBavg: " );
    lcd.print( b.vavg[1] ); lcd.setCursor(12,0); lcd.print(while_loop_counter);
    lcd.setCursor( 0,1 );
    lcd.print( " t: " );
    lcd.print( t_diffs[1] );
    lcd.print(" (");
    lcd.print( t_diffs[1] /NODE_VOLTAGES_ARRAY_SIZE );
    lcd.print("per)");
    delay(lcd_delay);
    lcd.setCursor( 0,0 ); lcd.print( "vBmax: " ); lcd.print( b.vmax[1] );
    delay(lcd_delay);
    lcd.setCursor( 0,0 ); lcd.print( "vBmin: " ); lcd.print( b.vmin[1] );
    delay(lcd_delay);

    lcd.clear();
    lcd.print( "vCavg: " );
    lcd.print( c.vavg[1] ); lcd.setCursor(12,0); lcd.print(while_loop_counter);
    lcd.setCursor( 0,1 );
    lcd.print( " t: " );
    lcd.print( t_diffs[2] );
    lcd.print(" (");
    lcd.print( t_diffs[2] /NODE_VOLTAGES_ARRAY_SIZE );
    lcd.print("per)");
    delay(lcd_delay);
    lcd.setCursor( 0,0 ); lcd.print( "vCmax: " ); lcd.print( c.vmax[1] );
    delay(lcd_delay);
    lcd.setCursor( 0,0 ); lcd.print( "vCmin: " ); lcd.print( c.vmin[1] );
    delay(lcd_delay);

    if ( ++while_loop_counter > 999 ) { while_loop_counter = 0; lcd.clear(); lcd.print("wtf is going on?"); delay(4000); }
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
void compute_node_voltages_vavg_vmin_vmax( node_voltages& node_voltages_struct, unsigned int data[], int struct_index ) {
  
  unsigned long node_sum = 0;
  node_voltages_struct.vmax[struct_index] = 0;    // start with default value
  node_voltages_struct.vmin[struct_index] = 1023; // start with default value
  
  int n;
  for ( n=0; n<DATA_ARRAY_SIZE; n++ ){
    node_sum += data[n];

    if ( node_voltages_struct.vmax[struct_index] < data[n] ) {
      node_voltages_struct.vmax[struct_index] = data[n];
    }

    if ( node_voltages_struct.vmin[struct_index] > data[n] ) {
      node_voltages_struct.vmin[struct_index] = data[n];
    }
  }
  node_voltages_struct.vavg[struct_index] = node_sum / DATA_ARRAY_SIZE;
}


void lcd_slow_dots( unsigned int tdelay, unsigned int dot_count) {
  dot_count = max( dot_count, 1 );          // better not be zero
  tdelay = max( min( tdelay/dot_count, 5000 ), 1 );   // max 5 seconds

  int n;
  for ( n=0; n<dot_count; n++ ) {
    lcd.print(".");
    delay( tdelay );
  }
}


// reports space between the heap and the stack
int freeRam () // https://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
