/**
 *             testTimerTwo
 *             testTimerTwo.ino
 * Purpose:    A test program to help develop a library for Timer2.
 *
 * @author:    Tyler Lucas
 * Student ID: 3305203
 * Date:       May 2, 2017
 * Version     1.0
 * 
 * References: http://maxembedded.com/2011/06/avr-timers-timer0/
 */

volatile unsigned long counter = 0;

void setup()
{
  pinMode( 13, OUTPUT );  // LED pin
  timer2_init();
}


void loop()
{
  // nada
}


void timer2_init()
{
  TCCR2A = 0;
  TCCR2B = (1 << CS21) | (1 << CS20); // prescale /8

  TIMSK2 = 1 << TOIE2;  // Timer2 overflow interrupt enable

  TCNT2 = 0; // load timer for 1st cycle

}

ISR(TIMER2_OVF_vect) {
  counter++;
  if ( counter > 1e3 )
  {
    digitalWrite( 13, !digitalRead(13) );
    counter = 0;
  }
}
