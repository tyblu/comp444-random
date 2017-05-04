/**
 *             testTimerTwo
 *             testTimerTwo.ino
 * Purpose:    A test program to help develop a library for Timer2.
 *
 * @author:    Tyler Lucas
 * Student ID: 3305203
 * Date:       May 3, 2017
 * Version     1.1
 * 
 * References: http://maxembedded.com/2011/06/avr-timers-timer0/
 *             http://www.uchobby.com/index.php/2007/11/24/arduino-interrupts/
 */

const unsigned int output_pin = 13;
unsigned int toggle_me_pls = LOW;

#define F_CPU 16000000
#define RESOLUTION 256

// TimerTwo stuff it needs
char oldSREG;
unsigned char timer2_prescale_bits;
unsigned long tcnt2;
unsigned long prescale_factor = 1;

// TimerTwo overflow counter variables
volatile unsigned long counter = 0;
volatile unsigned long counter_limit = (1 << 31); // 2^31

void setup()
{
  Serial.begin(9600);
  pinMode( output_pin, OUTPUT );
}


void loop()
{
  long timer2_period = 35;  // in [us], minimum 6 or program hangs, up to 40% error in /1
  timer2_init( timer2_period );
  long timer_delay = 220;  // in [us]
  counter_limit = timer_delay / timer2_period; // [us] / [us]

  Serial.println();
  Serial.print("  timer2_period: "); Serial.println(timer2_period);
  Serial.print("          tcnt2: "); Serial.println(tcnt2);
  Serial.print("      256-tcnt2: "); Serial.println(256-tcnt2);
  Serial.print("prescale factor: "); Serial.println(prescale_factor);
  Serial.print("     timer xtal: "); Serial.print((F_CPU/1000)/prescale_factor); Serial.println("kHz");
  Serial.print("     timer tick: "); Serial.print( (256-tcnt2) * prescale_factor / ( F_CPU / 1000000 ) ); Serial.println("us");
  Serial.print("    timer_delay: "); Serial.println(timer_delay);
  Serial.print("  counter_limit: "); Serial.println(counter_limit);

  timer2_start();

  unsigned long timestamp;

  int prev_state = digitalRead( output_pin ); 

  while ( true ) {
//    Serial.println("\nTimer2 enabled!");

    timer2_restart();
    timer2_enable();
//    timestamp = micros();

    while ( digitalRead( output_pin ) == prev_state ) { }

//    timestamp = micros() - timestamp;
//    timer2_stop();
    timer2_disable();
//    timer2_restart();

    prev_state = digitalRead( output_pin );

//    Serial.print("Timer2 disabled!"); Serial.print(" Time: "); Serial.print( timestamp ); Serial.println("us");

    delayMicroseconds( timer_delay*2 );
  }
}


void timer2_init( long microseconds )
{ 
  // prescale by /1 (16MHz, 16us OVF)
  TCCR2A = 0;   // Normal port operation, OC2A disconnected
  return timer2_set_period( microseconds );
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
  TCNT2 = tcnt2;
}

void timer2_start()
{
  unsigned int tcnt2_temp;

  TIMSK2 &= ~(1 << TOIE2);  // Timer2 OVF ISR disable
  
  oldSREG = SREG;
  cli();
  TCNT2 = tcnt2;    // restart counter
  SREG = oldSREG;
  timer2_restart();
  do { // wait until timer gets past it's first tick to stop 'phantom' interrupt
    oldSREG = SREG;
    cli();
    tcnt2_temp = TCNT2;
    SREG = oldSREG;
  } while ( tcnt2_temp == tcnt2 );

  TIFR2 = 0xFF;             // clear interrupt flags
  TIMSK2 |= (1 << TOIE2);   // Timer2 OVF ISR enable
}

void timer2_stop()    // stop timer, ISR not disabled (but won't fire)
{
  TCCR2B &= ~( (1 << CS22) | (1 << CS21) | (1 << CS20) ); // clears all clock select bits
}

void timer2_enable()  // enable ISR, timer may be anywhere
{
  TIFR2 = 0xFF;             // clear interrupt flags
  TIMSK2 |= (1 << TOIE2);   // Timer2 OVF ISR enable
}

void timer2_disable() // disable ISR, timer continues
{
  TIMSK2 = 0;
}

ISR(TIMER2_OVF_vect) {
  TCNT2 = tcnt2;
  counter++;
  toggle_me_pls = ( toggle_me_pls + 1 ) % 2;
  digitalWrite( 12, toggle_me_pls );
  if ( counter > counter_limit )
  {
    digitalWrite( output_pin, !digitalRead(output_pin) );
    counter = 0;
  }
}

void timer2_restart()
{
  TCCR2B |= timer2_prescale_bits;
  TCNT2 = tcnt2;
}

