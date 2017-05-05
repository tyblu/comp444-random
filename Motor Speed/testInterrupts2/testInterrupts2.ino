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
 * Version     1.4
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


// Define various ADC prescaler (https://goo.gl/qLdu2e)
const unsigned char prescale_016 = (1 << ADPS2);                                //   1 MHz ( 76.9 kHz) ~  13 us
const unsigned char prescale_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // 125 kHz ( 9.6 kHz) ~  104 us

 /* Determine datapoints req'd to capture voltages over 2 PWM periods,
 * given an ADC rate from the prescaler. My Arduino Duo (SparkFun 
 * RedBoard) runs at 16MHz, and the PWM is set to 490Hz. Prescaler
 * will probably be set to 16 for a 1MHz ADC, but no promises.
 * Remember that the DATA_ARRAY_SIZE has to be scaled with the ADC
 * speed to ensure the measurements don't take too long. At 90% duty,
 * over the maximum I would expect for any PWM-controlled motor and
 * way over the 60% maximum for our SIK M260, there are only 204
 * microseconds available to sample the emf, and at least half of 
 * that is too noisy to be useful (inductive spike from current dump
 * and subsequent ringing), leaving around 100 microseconds. This is
 * not done programatically (with the below formula) to ensure the
 * user knows exactly what this number means. (It's important!)
 * 
 * DATA_ARRAY_SIZE = 100 microseconds * F_CPU / prescaler / 13 = about 8
  */
#define DATA_ARRAY_SIZE 8               // number of measurements per waveform. 
#define DATA_STATISTICS_ARRAY_SIZE 100  // number of waveforms over which to aggregate statistics
volatile unsigned int vdata[DATA_ARRAY_SIZE] = {0}; // holds analog data, may be collected in ISR
volatile unsigned int waveform_counter = 0;         // number of PWM periods analyzed
volatile bool data_ready = false;                   // flag indicating that vdata has been updated


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

// TimerTwo overflow counter variables
volatile unsigned long timer2_counter = 0;
volatile unsigned long timer2_counter_limit = (1 << 31); // 2^31, a big number


void setup()
{
  // Setup pins
  pinMode( motor_pin, OUTPUT );
  pinMode( output_pin_extern_ISR, OUTPUT ); // debugging
  pinMode( output_pin_A, OUTPUT );          // debugging
  pinMode( output_pin_B, OUTPUT );          // debugging

  // Setup ADC
  ADCSRA &= ~prescale_128;    // remove bits set by Arduino library
  ADCSRA |= prescale_016;     // set our own prescaler (1MHz)

  // Setup serial link and spit out intro
  Serial.begin(9600);
  Serial.println("ISR Testing, 1MHz ADC");
  Serial.println("v1.4-20170505\n");
  delay(1500);
}


void loop()
{
  // Setup TimerTwo, do not enable ISR yet.
  // Start with 0.25 the period of a 490Hz PWM waveform.
  long timer2_period = 510; // 0.25 * ( 1/490Hz ) = 510 microseconds
  long timer_delay = 510;
  timer2_counter_limit = timer_delay / timer2_period;
  timer2_init( timer2_period );

  // Setup external ISR
  digitalWrite( motor_pin, LOW ); // prevent external ISR from firing before ready
  // Calls timer2_enable(), so the external ISR turns on Timer2 ISRs
  attachInterrupt( digitalPinToInterrupt( ISR_pin ), external_ISR, FALLING );

  // Print TimerTwo data out, to make sure everything looks right
  Serial.print("  timer2_period: "); Serial.print(timer2_period); Serial.println(" [us]");
  Serial.print("          tcnt2: "); Serial.println(tcnt2);
  Serial.print("      256-tcnt2: "); Serial.println(256-tcnt2);
  Serial.print("prescale factor: "); Serial.println(prescale_factor);
  Serial.print("     timer xtal: "); Serial.print((F_CPU/1000)/prescale_factor); Serial.println(" [kHz]");
  Serial.print("     timer tick: "); Serial.print( (256-tcnt2) * prescale_factor / ( F_CPU / 1000000 ) ); Serial.println(" [us]");
  Serial.print("    timer_delay: "); Serial.print(timer_delay); Serial.println(" [us]");
  Serial.print("  counter_limit: "); Serial.println(timer2_counter_limit);

  // Setup statistical variables
  int vdata_mean[100], vdata_std_dev2[100];
  int vdata_mean_mean, vdata_mean_std_dev2, vdata_std_dev2_mean, vdata_std_dev2_std_dev2;
  unsigned int waveform_counter = 0, waveform_counter_skipped = 0;
  
  // Get motor going, starting everything else.
  int duty_cycle = 45;
  analogWrite( motor_pin, ( 255 * duty_cycle )/100 );  // PWM starts, firing off external ISRs, chaining to Timer2 ISRs
  Serial.println("\nMotor should be spinning now...");
  delay(1000);

  bool is_break_time = false;

  while ( true )
  {
    while ( !is_break_time )
    { // put duty cycle-based motor variation here, later
      if ( data_ready )
      {
        
        
        /* Should I disable interrupts here? If I need to, it means that
         *  I'm missing the next period. This is okay, but not the
         *  intended timing of the program. Instead, I'll use the flag
         *  'data_ready' and keep the ISRs active. */
        // get average, std. dev.      
        vdata_mean[waveform_counter] = get_avg( vdata, DATA_ARRAY_SIZE );
        vdata_std_dev2[waveform_counter] = get_std_dev2( vdata, vdata_mean[waveform_counter], DATA_ARRAY_SIZE );

        /* Is this measured data any good? Is the variance or standard
         *  deviation within acceptable limits? Having viewed the
         *  oscilloscope data, I can see that the measured signal varies
         *  up to around 400 millivolts, only very rarely going outside.
         *  To me, this means 6 standard deviations (+3 up, -3 down),
         *  giving about a 67 mV standard deviation. */
        if ( vdata_std_dev2[waveform_counter] < 67 * 67 )
        {
          waveform_counter++; // data is acceptable, increment counter
        }
        else
        {
          waveform_counter_skipped++; // we skipped another waveform
        }

        if ( waveform_counter < DATA_STATISTICS_ARRAY_SIZE ) {  // need more data
          data_ready = false; // finished processing data, flag for new
        }
        else
        { // stats collected from many waveforms, time to get values
          is_break_time = true;
        }
      }
    }

    detachInterrupt( digitalPinToInterrupt( ISR_pin ) );
//    noInterrupts(); // not sure if this disables Timer2 ISR, but it will only fire once if not
    timer2_stop();

    // Aggregate statistics.
    vdata_mean_mean = get_avg( vdata_mean, DATA_STATISTICS_ARRAY_SIZE );
    vdata_mean_std_dev2 = get_std_dev2( vdata_mean, vdata_mean_mean, DATA_STATISTICS_ARRAY_SIZE );
    vdata_std_dev2_mean = get_avg( vdata_std_dev2, DATA_STATISTICS_ARRAY_SIZE );
    vdata_std_dev2_std_dev2 = get_std_dev2( vdata_std_dev2, vdata_std_dev2_mean, DATA_STATISTICS_ARRAY_SIZE );

    Serial.print("\nLet's take a breather! Whew, already ");
    Serial.print( waveform_counter + waveform_counter_skipped );
    Serial.println(" PWM waveforms passed since last zeroed!");
    Serial.print("We had to skip ");
    Serial.print( waveform_counter_skipped );
    Serial.print(" waveforms in order to get ");
    Serial.print( waveform_counter );
    Serial.println(" good sets of measurements.");
    Serial.println("Let's take a closer look at our numbers...");
    delay(500);

    Serial.print("\nThe average value over all waveforms was ");
    Serial.print( vdata_mean_mean );
    Serial.print(", and had an average standard deviation (squared) of ");
    Serial.print( vdata_std_dev2_mean );
    Serial.println(".");
    delay(500);

    Serial.print("The standard deviation (squared) of the average values was ");
    Serial.print( vdata_mean_std_dev2 );
    Serial.print(", and the standard deviation of this statistic was ");
    Serial.print( vdata_std_dev2_std_dev2 );
    Serial.println(".");
    delay(500);

    Serial.println("\nDoes that look right? Do you want to see the individual statistics?");
    Serial.println(" (Type \'y\' to see them, or any other characters or wait 3 sec to skip.)");

    long timestamp = millis() + 3000;
    while( Serial.available() == 0 && millis() < timestamp ) { } // wait for input

    if ( Serial.read() == 'y' )
    {
      print_all_stats( vdata_mean, vdata_std_dev2, DATA_STATISTICS_ARRAY_SIZE );
    }
    delay(500);

    Serial.println("\nDo you want to see the last waveform measurements?");
    Serial.println(" (Type \'y\' to see them, or any other characters or wait 3 sec to skip.)");

    timestamp = millis() + 3000;
    while( Serial.available() == 0 && millis() < timestamp ) { } // wait for input

    if ( Serial.read() == 'y' )
    {
      int n;
      for ( n=0; n<DATA_ARRAY_SIZE; n++ )
      {
        Serial.println(vdata[n]);
      }
    }
    delay(500);
    
    // debugging, ran out of SRAM a few times
//    Serial.println("\n"); Serial.print(freeRam()); Serial.println("FREE SRAM"); 
    
    Serial.println("\nI hope it worked! Restarting in 5 seconds. Send any character to stop testing.");

    timestamp = millis() + 5000;
    while( millis() < timestamp ) { if ( Serial.available() > 0 ) { stop_everything(); } }

    Serial.println("... Well, back at it!\n");
    delay(500);

    waveform_counter = 0;
    waveform_counter_skipped = 0;
    
    timer2_start(); // does not enable ISR, external ISR controls that
    attachInterrupt( digitalPinToInterrupt( ISR_pin ), external_ISR, FALLING );
//    interrupts();   // not sure if this enables Timer2 ISR, hopefully not
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


ISR( TIMER2_OVF_vect )
{
  digitalWrite( output_pin_A, HIGH );
  timer2_counter++;
  if ( timer2_counter >= timer2_counter_limit )
  {
    digitalWrite( output_pin_B, HIGH );
    int n;
    for ( n=0; n<DATA_ARRAY_SIZE; n++ ) {
      vdata[n] = analogRead( motor_neg_pin );
    }
    timer2_counter = 0;
    timer2_disable(); // Timer2 ISR is done, can be enabled again by external ISR
    data_ready = true; // vdata[] is ready for number crunching
    digitalWrite( output_pin_B, LOW );
  }
  digitalWrite( output_pin_A, LOW );
}


void external_ISR()
{
  digitalWrite( output_pin_extern_ISR, HIGH );
  
  if ( !data_ready )  // need new data
  {
    timer2_restart_zero();  // reset and start clock
    timer2_enable();        // enable Timer2 ISR
  }
//  else
//  {
//    // flag to let you know that you missed a period
//  }
  
  digitalWrite( output_pin_extern_ISR, LOW );
}


int get_avg( int data[], int data_size )
{
  long temp_avg = 0;
  int n;
  for ( n=0; n<data_size; n++ )
  {
    temp_avg += data[n];
  }
  return (int)( temp_avg / data_size );
}


int get_std_dev2( int data[], int data_mean, int data_size )
{
  long temp_std_dev = 0;
  int temp_var;
  int n;
  for ( n=0; n<data_size; n++ )
  {
    temp_var = data[n] - data_mean;
    temp_std_dev += temp_var * temp_var;
  }
  return (int)( temp_std_dev / data_size );
}


void print_all_stats( int avgs[], int stddev2s[], int stats_data_size )
{
  Serial.write('#');
  Serial.println("\taverage\tstd.dev.sq.");
  
  int n;
  for ( n=0; n<stats_data_size; n++ )
  {
    Serial.print( n+1 );
    Serial.print( '\t' );
    Serial.print( avgs[n] );
    Serial.print( '\t' );
    Serial.println( stddev2s[n] );
  }

  Serial.println();
}


void stop_everything()
{
  Serial.println("\n\nGoing to sleep now. Power cycle required.");
  cli();
//  sleep_enable();
//  sleep_cpu();
  while ( true ) { }  // I don't actually know how to put the Arduino to sleep.
}

