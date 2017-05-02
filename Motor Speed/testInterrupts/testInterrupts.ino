/**
 *             testInterrupts
 *             testInterrupts.ino
 * Purpose:    Experimenting with synchronizing PWM with ADC measurements
 *             using interrupts.
 *             
 *             Though not the immediate goal of this program, it will 
 *             allow the direct measurement of the back-emf produced by a
 *             small PWM-modulated DC motor during its "off" phase in the
 *             PWM waveform, and the calculation of its peak current (as
 *             well as RMS current) during its "on" phase.
 *
 * @author:    Tyler Lucas
 * Student ID: 3305203
 * Date:       May 2, 2017
 * Version     1.0
 * 
 * References: http://gammon.com.au/interrupts
 *             https://www.programiz.com/cpp-programming/examples/standard-deviation
 */

#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

const int motor_pin = 9; // PWM driving NPN to control motor
const int motor_pos_pin = A0;  // to motor +ve
const int motor_neg_pin = A1;  // to motor -ve and shunt top
const int collector_pin = A2;  // to shunt bottom and collector
const int ISR_pin = 2;  // external interrupt pin (#3 occupied by LCD)


// Define various ADC prescaler (https://goo.gl/qLdu2e)
const unsigned char prescale_002 = (1 << ADPS0);                                //   8 MHz (615.4 kHz) ~   1.6 us
const unsigned char prescale_004 = (1 << ADPS1);                                //   4 MHz (307.7 kHz) ~   3.3 us
const unsigned char prescale_008 = (1 << ADPS1) | (1 << ADPS0);                 //   2 MHz (153.8 kHz) ~   6.5 us
const unsigned char prescale_016 = (1 << ADPS2);                                //   1 MHz ( 76.9 kHz) ~  13 us
const unsigned char prescale_032 = (1 << ADPS2) | (1 << ADPS0);                 // 500 kHz ( 38.5 kHz) ~  26 us
const unsigned char prescale_064 = (1 << ADPS2) | (1 << ADPS1);                 // 250 kHz ( 19.2 kHz) ~  52 us
const unsigned char prescale_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // 125 kHz ( 9.6 kHz) ~  104 us


/* Determine datapoints req'd to capture voltages over 2 PWM periods,
 * given an ADC rate from the prescaler. My Arduino Duo (SparkFun 
 * RedBoard) runs at 16MHz, and the PWM is set to 490Hz. Prescaler
 * will probably be set to 16 for a 1MHz ADC, but no promises. Note
 * that this uses too much SRAM if the prescale is set to 2. In fact,
 * it's completely untested, but 2*16e6/(2*13*490) ints is 2.5kB on
 * its own, pushing this program way over the Duo's 2k SRAM. */
#define F_CPU 16e6        // 16MHz
#define F_PWM 490         // 490Hz
#define PRESCALE_ADC 16   // set to 1MHz for now
#define DATA_ARRAY_SIZE 313 //(int)(2 * F_CPU / ( PRESCALE_ADC * 13 * F_PWM )) // =313
#define DATA_ARRAY_WINDOW_SIZE 3 // (int)(40 * ( F_CPU/1e6 ) /( PRESCALE_ADC * 13 ) + 0.5) // 40us windows, ~~3
#define DATA_ARRAY_SUBWINDOW_SIZE 307 // (int)(2 * F_CPU / ( PRESCALE_ADC * 13 * F_PWM ) - 40 * ( F_CPU/1e6 ) /( PRESCALE_ADC * 13 ) + 0.5 )// DATA_ARRAY_SIZE - 2*DATA_ARRAY_WINDOW_SIZE was generating "overflow in constant expression" error
volatile unsigned int vdata[DATA_ARRAY_SIZE] = {0};  // for use in ISR


#define ISR_MODE RISING
volatile int next_analog_pin;                   // Which node to measure?
volatile bool need_waveform_update;             // Do we need to measure the next waveform?
volatile byte waveform_counter = 0;             // Keep track of how many waveforms we've measured in a row.
volatile const byte waveform_counter_max = 10;  // Analyze data from how many waveforms at a time?
unsigned int waveform_period_between_updates = 100;  // For how long to NOT do any measurements? [ms] (~100ms?)

//struct value {
//  int stddev;
//  int mean;
//}

//struct waveform {
//  value emf;   // avg of at least 2 values just before "on" period
//  value on;    // avg of at least 2 values just before "off" period
//}

const unsigned int lcd_delay = 3000;

void setup() {
  
  pinMode( motor_pin, OUTPUT );
  
  ADCSRA &= ~prescale_128;    // remove bits set by Arduino library
  ADCSRA |= prescale_016;     // set our own prescaler
  // Don't forget to set #define PRESCALE_ADC (line 44'ish).

  Serial.begin(9600);

  lcd.begin(16, 2); //Initialize the 16x2 LCD
  lcd.clear();
  lcd.print("Testing 16MHz/PRESCALE_ADC"); Serial.println("Testing 16MHz/PRESCALE_ADC");
  lcd.setCursor(0,1);
  lcd.print("v1.01-20170502"); Serial.println("v1.01-20170502\n");
  delay(1500);
  lcd.clear();
}

void loop() {

  Serial.println( freeRam() + " bytes free in stack." );
  lcd.print( freeRam() + " bytes free in stack." ); delay(lcd_delay);
  
  unsigned int motor_duty_byte = 153;      // 153/255 = 60% duty
  analogWrite( motor_pin, motor_duty_byte );
  lcd.clear(); lcd.print("Just a moment"); lcd_slow_dots(lcd_delay, 3); lcd.clear(); lcd.print("Runneratering."); // allow electronics to stabilize

  waveform_counter = 0;
  byte waveform_counter_last = 0;

  attachInterrupt( digitalPinToInterrupt( ISR_pin ), measure_waveform, ISR_MODE ); // enable ISR after stabilization

  while ( true ) {  // this main loop shouldn't have anything to do with ISRs
    if ( waveform_counter == 1 ) {
      Serial.println("waveform_counter == 1");
//      need_waveform_update = false;   // disabled while testing
      waveform_counter = 0;
      detachInterrupt( digitalPinToInterrupt( ISR_pin ) );      // disable ISR when we have enough data
//      t_next_waveform = millis() + waveform_period_between_updates;
      analyze_waveform( vdata );
      attachInterrupt( digitalPinToInterrupt( ISR_pin ), measure_waveform, ISR_MODE ); // debugging
    }

    if ( waveform_counter != waveform_counter_last ) {
      waveform_counter_last = waveform_counter;
      Serial.print("Yet another waveform measured! (#"); Serial.print(waveform_counter); Serial.println(")");
      delay(250); // don't want to spam monitor
    }
  }
}


// greatest function ever created
void lcd_slow_dots( unsigned int tdelay, unsigned int dot_count) {
  dot_count = max( dot_count, 1 );          // better not be zero
  tdelay = max( min( tdelay/dot_count, 5000 ), 1 );   // max 5 seconds

  int n;
  for ( n=0; n<dot_count; n++ ) {
    lcd.print(".");
    delay( tdelay );
  }
}


// find peak, emf ("off") and running voltages...
// or just print it all and go from there!
// this is full of debug code
void analyze_waveform( unsigned int data[] ) {

  unsigned int array_window[ DATA_ARRAY_WINDOW_SIZE ];    // 40us windows, ~~3
  unsigned int rolling_mean[ DATA_ARRAY_SUBWINDOW_SIZE ] = {0};
  unsigned int rolling_std_dev[ DATA_ARRAY_SUBWINDOW_SIZE ] = {0};
  
  int m, n;
  Serial.println( "RAW DATA " + millis() );  // debugging
  for ( n=0; n<DATA_ARRAY_SIZE; n++ ) {     // debugging
    Serial.println(data[n]);                // debugging
  }
  Serial.println( "\n\nSTATISTICS " + millis() );
  Serial.println( " Rolling Means" );
  for ( n=(DATA_ARRAY_WINDOW_SIZE-1); n<( DATA_ARRAY_SIZE - DATA_ARRAY_WINDOW_SIZE ); n++ ) {
    rolling_mean[n] += sum_from_here_to_there( data, n, n + DATA_ARRAY_WINDOW_SIZE-1 );
    rolling_mean[n] /= DATA_ARRAY_WINDOW_SIZE;  // get means of data windows
    Serial.println( rolling_mean[n] );
  }
  Serial.println( "\n Rolling Standard Deviations" );
  for ( n=(DATA_ARRAY_WINDOW_SIZE-1); n<( DATA_ARRAY_SIZE - DATA_ARRAY_WINDOW_SIZE ); n++ ) {
    for ( m=0; m<DATA_ARRAY_WINDOW_SIZE; m++ ) {
      rolling_std_dev[n] += pow( data[n+m] - rolling_mean[n], 2 ); // get std.dev. of data windows
    }
    Serial.println( rolling_std_dev[n] );
  }
}


// return sum of all elements of array between given indices, endpoints inclusive
unsigned long sum_from_here_to_there( unsigned int data[], int here, int there ){
  unsigned long sum = 0;
  
  int n;
  for ( n=here; n<=there; n++) {
    sum += data[n];
  }

  return sum;
}


void measure_waveform() {
  Serial.println("I\'m in!");                     // debugging
  int n;
  for ( n=0; n<DATA_ARRAY_SIZE; n++ ) {
    vdata[n] = analogRead( motor_neg_pin );
  }
  waveform_counter++;
  Serial.println("I\'m out!");                     // debugging
}


// reports space between the heap and the stack
int freeRam () // https://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
