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

#define F_CPU 16000000
#define RESOLUTION 256

// TimerTwo stuff it needs
char oldSREG;

// TimerTwo overflow counter variables
volatile unsigned long counter = 0;
volatile unsigned long counter_limit = (1 << 31); // 2^31

void setup()
{
  Serial.begin(9600);
  
  pinMode( 13, OUTPUT );  // LED pin
}


void loop()
{
  long timer2_period_requested = 8000;  // in [us]
  long timer2_period_returned = timer2_init( timer2_period_requested );
  long timer_delay = 500000;  // in [us]
  counter_limit = timer_delay / timer2_period_returned;

  Serial.println();
  Serial.print("timer2_period_requested: "); Serial.println(timer2_period_requested);
  Serial.print(" timer2_period_returned: "); Serial.println(timer2_period_returned);
  Serial.print("            timer_delay: "); Serial.println(timer_delay);
  Serial.print("          counter_limit: "); Serial.println(counter_limit);

  while ( true ) {
    timer2_enable();

    Serial.println("\nTimer2 enabled!");

    delay( 5000 );

    timer2_disable();

    Serial.println("Timer2 disabled!");

    delay( 5000 );
  }
}


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
  counter++;
  if ( counter > counter_limit )
  {
    digitalWrite( 13, !digitalRead(13) );
    counter = 0;
  }
}
