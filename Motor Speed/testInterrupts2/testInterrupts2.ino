/**
 *             testInterrupts2
 *             testInterrupts2.ino
 * Purpose:    Experimenting with synchronizing PWM with ADC measurements
 *             using interrupts.
 *             
 *             Though not the immediate goal of this program, it will 
 *             allow the direct measurement of the back-emf produced by a
 *             small PWM-modulated DC motor during its "off" phase in the
 *             PWM waveform, and the calculation of its peak current (as
 *             well as RMS current) during its "on" phase.
 *             
 *             This is a re-do of testInterrupts.ino, which is full of bugs.
 *
 * @author:    Tyler Lucas
 * Student ID: 3305203
 * Date:       May 2, 2017
 * Version     1.3
 * 
 * References: http://gammon.com.au/interrupts
 *             https://www.programiz.com/cpp-programming/examples/standard-deviation
 *             https://code.google.com/archive/p/arduino-timerone/
 *             http://www.uchobby.com/index.php/2007/11/24/arduino-interrupts/
 *             http://maxembedded.com/2011/06/avr-timers-timer0/
 */

/* Circuit diagram a-la-crap:
 *  
 * |------------|
 * |            |                          10k
 * |         A0-|--------------------|-----/\/\/-----|MOTOR NEG PIN (or any analog voltage source)
 * |  A         |                    \
 * |  R         |    10k             / 10k
 * |  D      13-|---/\/\/---|GND     \
 * |  U         |    10k            _|_
 * |  I      12-|---/\/\/---|GND    GND
 * |  N         |    10k
 * |  O      11-|---/\/\/---|GND   NOTE: PINS 11,12,13 FOR DEBUGGING ONLY.
 * |            |
 * |  D         |       330
 * |  U   PWM 9-|---|---/\/\/---|BJT BASE
 * |  E         |   /
 * |            |   \ 10k
 * | external   |   /
 * |      ISR 2-|---|
 * |            |
 * |------------|
 * 
 */

// Pin assignments.
const int motor_pin = 9; // PWM driving NPN to control motor
const int motor_neg_pin = A1;  // to motor -ve and shunt top
const int ISR_pin = 2;  // external interrupt pin (#3 occupied by LCD)
const unsigned int output_pin_extern_ISR = 13;  // for external ISR verification on osc.
const unsigned int output_pin_A = 12; // TimerTwo tick-tock
const unsigned int output_pin_B = 11; // TimerTwo, long tick-tock


// External ISR stuff it needs
volatile unsigned int toggle_me_extern_ISR = LOW; // for external ISR verification on oscilloscope


// Define various ADC prescaler (https://goo.gl/qLdu2e)
const unsigned char prescale_016 = (1 << ADPS2);                                //   1 MHz ( 76.9 kHz) ~  13 us
const unsigned char prescale_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // 125 kHz ( 9.6 kHz) ~  104 us

 /* Determine datapoints req'd to capture voltages over 2 PWM periods,
 * given an ADC rate from the prescaler. My Arduino Duo (SparkFun 
 * RedBoard) runs at 16MHz, and the PWM is set to 490Hz. Prescaler
 * will probably be set to 16 for a 1MHz ADC, but no promises. Note
 * that this uses too much SRAM if the prescale is set to 2. In fact,
 * it's completely untested, but 2*16e6/(2*13*490) ints is 2.5kB on
 * its own, pushing this program way over the Duo's 2k SRAM. */
#define DATA_ARRAY_SIZE 25
#define DATA_ARRAY_WINDOW_SIZE 3
#define DATA_ARRAY_SUBWINDOW_SIZE 25-6
volatile unsigned int vdata[DATA_ARRAY_SIZE] = {0}; // holds analog data, may be collected in ISR
volatile unsigned int waveform_counter = 0;         // number of PWM periods to analyze at once


/*  TimerTwo
 *  Adapted from https://code.google.com/archive/p/arduino-timerone/ and
 *  http://maxembedded.com/2011/06/avr-timers-timer0/.
 */

// TimerTwo stuff it needs
#define F_CPU 16000000
#define RESOLUTION 256
char oldSREG;
unsigned char timer2_prescale_bits;
unsigned long tcnt2;
unsigned long prescale_factor = 1;
volatile unsigned int toggle_me_A = LOW;  // TimerTwo, tick-tock
volatile unsigned int toggle_me_B = LOW;  // TimerTwo, long tick-tock
// toggle_me_A,B not used if ISRs are doing the ADC work

// TimerTwo overflow counter variables
volatile unsigned long counter = 0;
volatile unsigned long counter_limit = (1 << 31); // 2^31, a big number


void setup()
{
  // Setup pins
  pinMode( motor_pin, OUTPUT );
  pinMode( output_pin_extern_ISR, OUTPUT ); // debugging
  pinMode( output_pin_A, OUTPUT );          // debugging
  pinMode( output_pin_B, OUTPUT );          // debugging

  // Setup ADC
  ADCSRA &= ~prescale_128;    // remove bits set by Arduino library
  ADCSRA |= prescale_016;     // set our own prescaler

  // Setup serial link and spit out intro
  Serial.begin(9600);
  Serial.println("ISR Testing, 1MHz ADC");
  Serial.println("v1.3-20170504\n");
  delay(1500);
}


void loop()
{
  // Setup TimerTwo, do not enable ISR yet.
  // Start with 0.25 the period of a 490Hz PWM waveform.
  long timer2_period = 510; // 0.25 * ( 1/490Hz ) = 510 microseconds
  long timer_delay = 510;
  counter_limit = timer_delay / timer2_period;
  timer2_init( timer2_period );

  // Setup external ISR
  digitalWrite( motor_pin, LOW ); // prevent external ISR from firing before ready
  // Calls timer2_enable(), so the external ISR turns on Timer2 ISRs
  attachInterrupt( digitalPinToInterrupt( ISR_pin ), external_ISR, RISING );

  // Print TimerTwo data out, to make sure everything looks right
  Serial.print("  timer2_period: "); Serial.print(timer2_period); Serial.println(" [us]");
  Serial.print("          tcnt2: "); Serial.println(tcnt2);
  Serial.print("      256-tcnt2: "); Serial.println(256-tcnt2);
  Serial.print("prescale factor: "); Serial.println(prescale_factor);
  Serial.print("     timer xtal: "); Serial.print((F_CPU/1000)/prescale_factor); Serial.println(" [kHz]");
  Serial.print("     timer tick: "); Serial.print( (256-tcnt2) * prescale_factor / ( F_CPU / 1000000 ) ); Serial.println(" [us]");
  Serial.print("    timer_delay: "); Serial.print(timer_delay); Serial.println(" [us]");
  Serial.print("  counter_limit: "); Serial.println(counter_limit);

  analogWrite( motor_pin, 153 );  // PWM starts, firing off external ISRs, chaining to Timer2 ISRs

  while ( true )
  {
    while ( 

    
    if ( waveform_counter > 5*490 )   // more than 5 seconds of 490Hz PWM
    {
      
      detachInterrupt( digitalPinToInterrupt( ISR_pin ) );
      Serial.print("\nLet's take a breather! Whew, already ");
      Serial.print( waveform_counter );
      Serial.println(" waveforms into it!");
      Serial.println("Let's take a closer look at the last waveform...");
  
      int n;
      for ( n=0; n<DATA_ARRAY_SIZE; n++ )
      {
        Serial.println(vdata[n]);
      }
      
      Serial.print("\nSend any character to continue...");
      while( !Serial.findUntil( "Go", '!' ) ) { delay(100); }
      Serial.println("... Well, back at it!");
      delay(1000);
      
      attachInterrupt( digitalPinToInterrupt( ISR_pin ), measure_waveform, RISING );
    }
  }
}


// reports space between the heap and the stack
int freeRam () // https://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}



// TimerTwo
// returns actual period
long timer2_init( long microseconds )
{ 
  // prescale by /1 (16MHz, 16us OVF)
  TCCR2A = 0;   // Normal port operation, OC2A disconnected
  return timer2_set_period( microseconds );
}

// returns actual period
long timer2_set_period( long microseconds )
{
  unsigned char prescale_bits;
  long prescale_factor;
  long cycles = ( F_CPU / 1000000 ) * microseconds;
  if ( cycles < RESOLUTION )            
  {
    prescale_bits = (1 << CS20);                 // no prescale, /1
    prescale_factor = 1;
  }
  else if ( (cycles>>=3) < RESOLUTION )
  {
    prescale_bits = (1 << CS21);                              // /8
    prescale_factor = 8;
  }
  else if ( (cycles>>=2) < RESOLUTION ) 
  {
    prescale_bits = (1 << CS21) | (1 << CS20);                // /32
    prescale_factor = 32;
  }
  else if ( (cycles>>=1) < RESOLUTION ) 
  {
    prescale_bits = (1 << CS22);                              // /64
    prescale_factor = 64;
  }
  else if ( (cycles>>=1) < RESOLUTION ) 
  {
    prescale_bits = (1 << CS22) | (1 << CS20);                // /128
    prescale_factor = 128;
  }
  else if ( (cycles>>=3) < RESOLUTION ) 
  { 
    prescale_bits = (1 << CS22) | (1 << CS21) | (1 << CS20);  // /1024
    prescale_factor = 1024;
  }
  else
  {
    prescale_bits = (1 << CS22) | (1 << CS21) | (1 << CS20);  // not enuf bits sir, max set
    prescale_factor = 1024;
  }

  TCCR2B &= ~( (1 << CS22) | (1 << CS21) | (1 << CS20) );
  TCCR2B |= prescale_bits;

  return ( RESOLUTION * prescale_factor ) / ( F_CPU / 1000000 );
}

void timer2_enable()
{
  TIMSK2 = (1 << TOIE2);  // Timer2 OVF ISR enable
  oldSREG = SREG;
  cli();
  TCNT2 = 0;    // restart counter
  SREG = oldSREG;
}

void timer2_disable()
{
  TIMSK2 = 0;
}

ISR(TIMER2_OVF_vect) {
  digitalWrite( output_pin_A, HIGH );
  counter++;
  if ( counter >= counter_limit )
  {
    digitalWrite( output_pin_B, HIGH );
    int n;
    for ( n=0; n<DATA_ARRAY_SIZE; n++ ) {
      vdata[n] = analogRead( motor_neg_pin );
    }
    counter = 0;
    timer2_disable(); // Timer2 ISR is done, can be enabled again by external ISR
    digitalWrite( output_pin_B, LOW );
  }
  digitaWrite( output_pin_A, LOW );
}

void external_ISR()
{
  digitalWrite( output_pin_extern_ISR, toggle_me_extern_ISR^=1 );
  timer2_restart_zero();  // reset and start clock
  timer2_enable();        // enable Timer2 ISR
}

