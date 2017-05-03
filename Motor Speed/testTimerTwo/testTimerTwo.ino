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

// TimerTwo prescaler
const unsigned char prescale_TimerTwo_mask = (1 << CS22) | (1 << CS21) | (1 << CS20);
const unsigned char prescale_TimerTwo_0001 = (1 << CS20);
const unsigned char prescale_TimerTwo_0008 = (1 << CS21);

// TimerTwo overflow counter variables
volatile unsigned long counter = 0;
volatile unsigned long counter_limit;

void setup()
{
  pinMode( 13, OUTPUT );  // LED pin
  timer2_init();
}


void loop()
{
//  unsigned int ISR_delay_us = 220;   // 220us
  unsigned long ISR_delay_us = 0.5*1e6; // in [us]
  counter_limit = ISR_delay_us / 16;

  unsigned long tstop = millis() + 5000;

  while ( true ) {
    if ( millis() > tstop ) {
      TIMSK2 = 0;
      while ( true ) { }
    }
  }
}


void timer2_init()
{ 
  // prescale by /1 (16MHz, 16us OVF)
  TCCR2A = 0;
  TCCR2B = prescale_TimerTwo_0001;

  TIMSK2 = 1 << TOIE2;  // Timer2 overflow interrupt enable

  TCNT2 = 0; // load timer for 1st cycle
}

ISR(TIMER2_OVF_vect) {
  counter++;
  if ( counter > counter_limit )
  {
    digitalWrite( 13, !digitalRead(13) );
    counter = 0;
  }
}
