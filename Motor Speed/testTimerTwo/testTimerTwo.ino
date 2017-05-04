/**
 *             testTimerTwo
 *             testTimerTwo.ino
 * Purpose:    A test program to help develop a library for Timer2.
 *
 * @author:    Tyler Lucas
 * Student ID: 3305203
 * Date:       May 3, 2017
 * Version     1.2
 * 
 * References: http://maxembedded.com/2011/06/avr-timers-timer0/
 *             http://www.uchobby.com/index.php/2007/11/24/arduino-interrupts/
 */
// Define only one of the below modes.
//#define OSC_ONLY // for doing measurements with oscilloscope, without serial link
#define SERIAL       // for doing measurements with micros(), with serial link
//#define OSC_SERIAL   // for doing measurements with oscilloscope, with serial link

/* Change these to experiment. Oscilloscope can less than 1 microsecond,
 * humans can see over 67000 microseconds (15 Hz). */
#ifdef SERIAL
#define TIMER_DELAY 500000        // slow down LED blink so you can see it
#define TIMER_DELAY_TEST_LOOPS 10 // test time will be TIMER_DELAY_TEST_LOOPS * TIMER_DELAY
const int loop_delay_time = 5;    // in [seconds]
#else
#define TIMER_DELAY 220
#define TIMER_DELAY_TEST_LOOPS 1
#endif

// Pin assignments.
const unsigned int output_pin_A = 12; // flips every overflow
volatile unsigned int toggle_me_A = LOW;
const unsigned int output_pin_B = 13; // flips every TIMER_DELAY
volatile unsigned int toggle_me_B = LOW;

#define F_CPU 16000000
#define RESOLUTION 256

// TimerTwo stuff it needs
char oldSREG;
unsigned char timer2_prescale_bits;
unsigned long tcnt2;
unsigned long prescale_factor = 1;

// TimerTwo overflow counter variables
volatile unsigned long counter = 0;
volatile unsigned long counter_limit = (1 << 31); // 2^31, a big number

void setup()
{
  
#ifndef OSC_ONLY
  Serial.begin(9600);
#endif

  pinMode( output_pin_A, OUTPUT );
  pinMode( output_pin_B, OUTPUT );
}


void loop()
{
  /* Period needs to be over 6us or the program will hang,
   *  spending all of its time in the ISRs. Aggregate delay
   *  time, timer_delay, has up to 40% error with /1
   *  prescale when measured with micros(), perhaps due to
   *  inaccuracy and overhead associated with using micros()
   *  and reading pin states (digitalRead).
   */
  long timer2_period = 220;  // in [us], minimum 6 or program hangs, up to 40% error in /1
  timer2_init( timer2_period );
  long timer_delay = TIMER_DELAY;  // in [us], make >15Hz if you want to see LED blink
  counter_limit = timer_delay / timer2_period; // [us] / [us]

#ifndef OSC_ONLY
  Serial.println();
  Serial.print("  timer2_period: "); Serial.print(timer2_period); Serial.println(" [us]");
  Serial.print("          tcnt2: "); Serial.println(tcnt2);
  Serial.print("      256-tcnt2: "); Serial.println(256-tcnt2);
  Serial.print("prescale factor: "); Serial.println(prescale_factor);
  Serial.print("     timer xtal: "); Serial.print((F_CPU/1000)/prescale_factor); Serial.println(" [kHz]");
  Serial.print("     timer tick: "); Serial.print( (256-tcnt2) * prescale_factor / ( F_CPU / 1000000 ) ); Serial.println(" [us]");
  Serial.print("    timer_delay: "); Serial.print(timer_delay); Serial.println(" [us]");
  Serial.print("  counter_limit: "); Serial.println(counter_limit);
#endif

  unsigned int prev_state = toggle_me_B;
  
  timer2_start();

#ifdef SERIAL
  unsigned long timestamp, timestamp_offset;

  /* This attempts to remove the timing error introduced by overhead
   *  in the loop checking for the toggled bit.
   */
  prev_state = toggle_me_A;
  
  int n; // removed from for loop to get outside of micros() timer
  
  timestamp = micros();
  for ( n=1; n<2; n++ )
  {
    while ( toggle_me_A == prev_state ) { test_while_loop(); asm(""); }
    asm("");
  }
  timestamp_offset = micros() - timestamp;
  
  // undo changes from this test
  counter--; 
  digitalWrite( output_pin_A, toggle_me_A^=1 );
  prev_state = toggle_me_B;
  Serial.print("\nTimestamp offset: "); Serial.print(timestamp_offset); Serial.println(" [us] per loop.\n");
#endif


  while ( true ) {
#ifdef SERIAL
    Serial.println("\nTimer2 enabled!");

    int n; // removed from for loop to get outside of micros() timer

    timer2_restart();
    timestamp = micros();
    timer2_enable();

    for ( n=0; n<TIMER_DELAY_TEST_LOOPS; n++ )
    {
      while ( toggle_me_B == prev_state ) { }
      prev_state = toggle_me_B;
    }
    
    timestamp = micros() - timestamp - TIMER_DELAY_TEST_LOOPS * timestamp_offset;
    
    timer2_disable();
    timer2_stop();
//    timer2_restart();

    prev_state = toggle_me_B;

    Serial.print("Timer2 disabled!");
    Serial.print(" Time: ");
    Serial.print( float(timestamp/1e6), 4 );
    Serial.print(" seconds. That is ");
    if ( timestamp > TIMER_DELAY_TEST_LOOPS * TIMER_DELAY ) { Serial.print( "+" ); }
    Serial.print( 100*( (float)(timestamp) / (float)(TIMER_DELAY_TEST_LOOPS * TIMER_DELAY) )-100, 3 );
    Serial.println("% from the target delay.");
    Serial.print("Timer restarting in "); Serial.print(loop_delay_time); Serial.println(" seconds...");
    
    delay( loop_delay_time*1000 );
#endif
  }
}


void timer2_init( long microseconds )
{ 
  // prescale by /1 (16MHz, 16us OVF)
  TCCR2A = 0;   // Normal port operation, OC2A disconnected
  timer2_set_period( microseconds );
}

void timer2_set_period( long microseconds )
{
  if ( microseconds * ( F_CPU / 1000000 ) < RESOLUTION * 1 )
  {
    timer2_prescale_bits = (1 << CS20);                 // no prescale, /1
    prescale_factor = 1;
  }
  else if ( microseconds * ( F_CPU / 1000000 ) < RESOLUTION * 8 )
  {
    timer2_prescale_bits = (1 << CS21);                              // /8
    prescale_factor = 8;
  }
  else if ( microseconds * ( F_CPU / 1000000 ) < RESOLUTION * 32 )
  {
    timer2_prescale_bits = (1 << CS21) | (1 << CS20);                // /32
    prescale_factor = 32;
  }
  else if ( microseconds * ( F_CPU / 1000000 ) < RESOLUTION * 64 )
  {
    timer2_prescale_bits = (1 << CS22);                              // /64
    prescale_factor = 64;
  }
  else if ( microseconds * ( F_CPU / 1000000 ) < RESOLUTION * 128 )
  {
    timer2_prescale_bits = (1 << CS22) | (1 << CS20);                // /128
    prescale_factor = 128;
  }
  else if ( microseconds * ( F_CPU / 1000000 ) < RESOLUTION * 1024 )
  { 
    timer2_prescale_bits = (1 << CS22) | (1 << CS21) | (1 << CS20);  // /1024
    prescale_factor = 1024;
  }
  else
  {
    timer2_prescale_bits = (1 << CS22) | (1 << CS21) | (1 << CS20);  // not enuf bits sir, max set
    prescale_factor = 1024;
  }

  TCCR2B &= ~( (1 << CS22) | (1 << CS21) | (1 << CS20) );
  TCCR2B |= timer2_prescale_bits;

  tcnt2 = RESOLUTION - ( microseconds * ( F_CPU / 1000000 ) ) / prescale_factor;
  TCNT2 = tcnt2;    // starts timer
}


void timer2_start() // don't call before timer2_init; disables ISR, call timer2_enable after
{
  unsigned int tcnt2_temp;

  TIMSK2 &= ~(1 << TOIE2);  // Timer2 OVF ISR disable
  
  oldSREG = SREG;
  cli();
  TCNT2 = tcnt2;    // restart counter
  SREG = oldSREG;
  timer2_restart(); // prescale determined earlier with timer2_init or timer2_set_period
  do { // wait until timer gets past it's first tick to stop 'phantom' interrupt
    oldSREG = SREG;
    cli();
    tcnt2_temp = TCNT2;
    SREG = oldSREG;
  } while ( tcnt2_temp == tcnt2 );

//  TIFR2 = 0xFF;             // clear interrupt flags
//  TIMSK2 |= (1 << TOIE2);   // Timer2 OVF ISR enable
}


void timer2_stop()    // stop timer, ISR not disabled (but won't fire)
{
  TCCR2B &= ~( (1 << CS22) | (1 << CS21) | (1 << CS20) ); // clears all clock select bits
}


void timer2_restart() // starts clock; does not zero timer, use timer2_restart_zero for that
{
  TCCR2B |= timer2_prescale_bits;
}


void timer2_restart_zero()  // resets timer back to tcnt2, starts clock
{
//  TIMSK2 &= ~(1 << TOIE2);  // Timer2 OVF ISR disable
  
  oldSREG = SREG;
  cli();
  TCNT2 = tcnt2;    // restart counter
  SREG = oldSREG;
  timer2_restart(); // prescale determined earlier with timer2_init or timer2_set_period
  
  // may be phantom interrupt here, clearing flag instead of waiting for timer tick
  TIFR2 = 0xFF;             // clear interrupt flag
  
//  TIMSK2 |= (1 << TOIE2);   // Timer2 OVF ISR enable
}


void timer2_enable()  // enables ISR, does not touch timer
{
  TIFR2 = 0xFF;             // clear interrupt flag
  TIMSK2 |= (1 << TOIE2);   // Timer2 OVF ISR enable
}


void timer2_disable() // disables ISR, clears interrupt flag
{
  TIMSK2 = 0;
  TIFR2 = 0xFF;             // clear interrupt flag
}


// the test ISR
ISR(TIMER2_OVF_vect) {
  timer2_restart_zero();  // restart and zero timer
  counter++;
//  toggle_me_A ^= 1;    // toggles between 0 and 1 (LOW or HIGH), breaks for other numbers
  digitalWrite( output_pin_A, toggle_me_A^=1 );
  if ( counter+1 >= counter_limit )
  {
//    toggle_me_B ^= 1;  // toggles between 0 and 1 (LOW or HIGH), breaks for other numbers
    digitalWrite( output_pin_B, toggle_me_B^=1 );
    counter = 0;
  }
}

void test_while_loop()
{
  timer2_restart_zero();
  counter++;
  digitalWrite( output_pin_A, toggle_me_A^=1 );
  volatile bool isFalse = false;
  if ( isFalse ) while (true) { asm(""); }
}

