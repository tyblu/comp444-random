/**
 *             setSpeedReadEmf
 *             setSpeedReadEmf.ino
 * Purpose:    Experimenting with synchronizing PWM with ADC measurements
 *             using interrupts. Measures voltage signals related to the 
 *             DC motor's emf during specific intervals within the driving
 *             PWM cycle, then calculates current and emf. Unlike many
 *             examples available online, this implementation does not 
 *             require 'pausing' the drive signals.
 *             
 *             Modeled after testTimerTwo and testInterrupts2.
 *
 * @author:    Tyler Lucas
 * Student ID: 3305203
 * Date:       May 2, 2017
 * Version     2.00
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
 * |            |                                                      10k
 * |         A0-|-----------------------------------------------|-----/\/\/-----|MOTOR POS PIN
 * |            |                        10k                    \
 * |         A1-|-----------------|-----/\/\/--|MOTOR NEG PIN   / 10k
 * |            |                 \                             \
 * |         12-|---|LCD RS       / 10k                        _|_
 * |         11-|---|LCD ENABLE   \                            GND
 * |          5-|---|LCD D4      _|_
 * |          4-|---|LCD D5      GND
 * |          3-|---|LCD D6         
 * |          2-|---|LCD D7
 * |  A         |           
 * |  R         |    10k       
 * |  D       8-|---/\/\/---|GND
 * |  U         |    10k 
 * |  I       7-|---/\/\/---|GND
 * |  N         |    10k
 * |  O       6-|---/\/\/---|GND   NOTE: PINS 6,7,8 FOR DEBUGGING ONLY.
 * |            |
 * |  D         |        330
 * |  U   PWM 9-|---|---/\/\/---|BJT BASE
 * |  E         |   /
 * |            |   \ 10k
 * | external   |   /
 * |      ISR 2-|---|
 * |            |
 * |         12-|---|LCD RS
 * |         11-|---|LCD ENABLE
 * |          5-|---|LCD D4
 * |          4-|---|LCD D5
 * |          3-|---|LCD D6
 * |          2-|---|LCD D7
 * |            |
 * |------------|
 * 
 */


// Pin assignments.
const int motor_pin = 9;      // PWM driving NPN to control motor
const int motor_pos_pin = A0; // to motor +ve and power supply
const int motor_neg_pin = A1; // to motor -ve and shunt top
const int ISR_pin = 2;        // external interrupt pin (#3 occupied by LCD)
const unsigned int output_pin_extern_ISR = 8;  // for external ISR verification on osc.
const unsigned int output_pin_A = 7; // TimerTwo tick-tock
const unsigned int output_pin_B = 6; // TimerTwo, long tick-tock


#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);  // ...(rs, enable, d4, d5, d6, d7)
char str_intro[] = "setSpeedReadEmf";
char str_version[] = "v2.00";


// Define various ADC prescaler (https://goo.gl/qLdu2e)
const unsigned char prescale_016 = (1 << ADPS2);                                //   1 MHz ( 76.9 kHz) ~  13 us
const unsigned char prescale_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // 125 kHz ( 9.6 kHz) ~  104 us


 /* Determine datapoints req'd to capture sufficient data,
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
// holds analog data, may be collected in ISR
volatile unsigned int v_pos_a[DATA_ARRAY_SIZE] = {0};  // motor +ve from 1st half of period
volatile unsigned int v_pos_b[DATA_ARRAY_SIZE] = {0};  // motor +ve from 1st half of period
volatile unsigned int v_neg_a[DATA_ARRAY_SIZE] = {0};  // motor -ve from 1st half of period
volatile unsigned int v_neg_b[DATA_ARRAY_SIZE] = {0};  // motor -ve from 1st half of period
volatile unsigned int waveform_counter = 0;         // number of PWM periods analyzed
volatile bool data_ready = false; // Finished reading data from entire period
volatile bool is_break_time = false;  // Flag to stop ISRs from clobbering data without or before disabling them
/* next_read_state == 0: 1st (HIGH) part of PWM period, motor +ve pin
 *                 == 1: 2nd (LOW) part of PWM period, motor +ve pin
 *                 == 2: 1st (HIGH) part of PWM period, motor -ve pin
 *                 == 3: 2nd (LOW) part of PWM period, motor -ve pin
 * 
 * Remember not to analogRead() for ~100us after switching
 * ADC pins to avoid incorporating noise from the ADC pin
 * multiplexer into the next measurement.
 */
volatile byte next_read_state = 0;


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


// Motor constants
#define MOTOR_DUTY_MAX 63   // 100 * 3V / (5V - 0.2V)
#define MOTOR_DUTY_MIN 21   // 100 * 1V / (5V - 0.2V)
const int motor_R_mOhm = 1950;  // 1.95 Ohm in [mOhm]
// Measured datapoints
#define MOTOR_RPM1 2400
#define MOTOR_EMF1 1200 // in [mV]
#define MOTOR_RPM2 6120
#define MOTOR_EMF2 3000 // in [mV]
#define MOTOR_TORQUE 5000 // in [mN*cm]
#define MOTOR_CURRENT 850 // in [mA]


void setup()
{
  // Setup pins
  pinMode( motor_pin, OUTPUT );
  pinMode( output_pin_extern_ISR, OUTPUT ); // debugging
  pinMode( output_pin_A, OUTPUT );          // debugging
  pinMode( output_pin_B, OUTPUT );          // debugging
  // ADC pins do not need setup

  // Setup ADC
  ADCSRA &= ~prescale_128;    // remove bits set by Arduino library
  ADCSRA |= prescale_016;     // set our own prescaler (1MHz)

  // Setup serial link and spit out intro
  Serial.begin(9600);
  Serial.println( str_intro );
  Serial.println( str_version );

  // LCD already setup, spit out intro
  lcd.begin(16, 2); //Initialize the 16x2 LCD
  lcd.clear();  // clear any old data displayed on the LCD
  lcd.print( str_intro );
  lcd.setCursor(0,1);
  lcd.print( str_version );
  
  delay(1500);
}


void loop()
{
  // Setup TimerTwo, do not enable ISR yet.
  // Start with 0.25 the period of a 490Hz PWM waveform.
  long timer2_period = 510; // 0.25 * ( 1/490Hz ) = 510 microseconds
  long timer_delay = 510;
  timer2_counter_limit = timer_delay / timer2_period; // =1
  timer2_init( timer2_period );

  // Setup external ISR
  digitalWrite( motor_pin, LOW ); // prevent external ISR from firing before ready
  // Calls timer2_enable(), so the external ISR turns on Timer2 ISRs
  attachInterrupt( digitalPinToInterrupt( ISR_pin ), external_ISR, FALLING );

  // Print TimerTwo data out to serial, to make sure everything looks right
  Serial.print("  timer2_period: "); Serial.print(timer2_period); Serial.println(" [us]");
  Serial.print("          tcnt2: "); Serial.println(tcnt2);
  Serial.print("      256-tcnt2: "); Serial.println(256-tcnt2);
  Serial.print("prescale factor: "); Serial.println(prescale_factor);
  Serial.print("     timer xtal: "); Serial.print((F_CPU/1000)/prescale_factor); Serial.println(" [kHz]");
  Serial.print("     timer tick: "); Serial.print( (256-tcnt2) * prescale_factor / ( F_CPU / 1000000 ) ); Serial.println(" [us]");
  Serial.print("    timer_delay: "); Serial.print(timer_delay); Serial.println(" [us]");
  Serial.print("  counter_limit: "); Serial.println(timer2_counter_limit);

  // Setup statistical variables
  int v_pos_a_mean[DATA_STATISTICS_ARRAY_SIZE], v_pos_a_std_dev2[DATA_STATISTICS_ARRAY_SIZE];
  int v_pos_b_mean[DATA_STATISTICS_ARRAY_SIZE], v_pos_b_std_dev2[DATA_STATISTICS_ARRAY_SIZE];
  int v_neg_a_mean[DATA_STATISTICS_ARRAY_SIZE], v_neg_a_std_dev2[DATA_STATISTICS_ARRAY_SIZE];
  int v_neg_b_mean[DATA_STATISTICS_ARRAY_SIZE], v_neg_b_std_dev2[DATA_STATISTICS_ARRAY_SIZE];
  int v_pos_a_mean_mean, v_pos_a_std_dev2_mean;
  int v_pos_b_mean_mean, v_pos_b_std_dev2_mean;
  int v_neg_a_mean_mean, v_neg_a_std_dev2_mean;
  int v_neg_b_mean_mean, v_neg_b_std_dev2_mean;
  unsigned int waveform_counter = 0, waveform_counter_skipped = 0;

  // Setup motor sweep loop
  int duty_cycle = MOTOR_DUTY_MIN; // in [%], 21 to 63 for M260 motor
  int ramp_dir = 1; // +1 or -1, motor ramping direction
  // How quickly to ramp up motor duty? [ms/%]
  const unsigned long motor_sweep_rate = 100;   // Ramp time ~4s for 100ms delay

  // Setup data we're after, initialized to -1 for error detection (by user)
  int emf_mV = -1;      // in [mV]
  int current_mA = -1;  // in [mA]
  int rpm = -1;         // in [rpm]
  int torque = -1;      // in [mN * cm] ([milli-Newton centimeters])

  // Setup LCD and serial output strings
  char str0[] = "emf:XXXX  I:XXXX";
  char str1[] = "rpm:XXXX t:XXXXX";
  // How often to update info for user? [ms/printout]
  const unsigned long user_update_rate = 1000;
  unsigned long timestamp = millis() + user_update_rate;

  while ( true )
  {
    while ( duty_cycle <= MOTOR_DUTY_MAX && duty_cycle >= MOTOR_DUTY_MIN )
    {
      analogWrite( motor_pin, ( 255 * duty_cycle )/100 );
      delay( motor_sweep_rate );

      duty_cycle += ramp_dir;

      if ( data_ready )
      {
        // get average, std. dev.
        v_pos_a_mean[waveform_counter] = get_avg( v_pos_a, DATA_ARRAY_SIZE );
        v_pos_b_mean[waveform_counter] = get_avg( v_pos_b, DATA_ARRAY_SIZE );
        v_neg_a_mean[waveform_counter] = get_avg( v_neg_a, DATA_ARRAY_SIZE );
        v_neg_b_mean[waveform_counter] = get_avg( v_neg_b, DATA_ARRAY_SIZE );
        v_pos_a_std_dev2[waveform_counter] = get_std_dev2( v_pos_a, v_pos_a_mean[waveform_counter], DATA_ARRAY_SIZE );
        v_pos_b_std_dev2[waveform_counter] = get_std_dev2( v_pos_b, v_pos_b_mean[waveform_counter], DATA_ARRAY_SIZE );
        v_neg_a_std_dev2[waveform_counter] = get_std_dev2( v_neg_a, v_neg_a_mean[waveform_counter], DATA_ARRAY_SIZE );
        v_neg_b_std_dev2[waveform_counter] = get_std_dev2( v_neg_b, v_neg_b_mean[waveform_counter], DATA_ARRAY_SIZE );

        /* Is this measured data any good? Is the variance or standard
         *  deviation within acceptable limits? Having viewed the
         *  oscilloscope data, I can see that the measured signal varies
         *  up to around 400 millivolts, only very rarely going outside.
         *  To me, this means 6 standard deviations (+3 up, -3 down),
         *  giving about a 67 mV standard deviation. */
        if ( v_pos_a_std_dev2[waveform_counter] < 67 * 67
          && v_pos_b_std_dev2[waveform_counter] < 67 * 67
          && v_neg_a_std_dev2[waveform_counter] < 67 * 67
          && v_neg_b_std_dev2[waveform_counter] < 67 * 67 )
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
        else  // Stats collected from many waveforms, time to get values.
        {
          // Flag ISRs to not collect data, etc., and then stop ISRs and timer
          is_break_time = true;
          detachInterrupt( digitalPinToInterrupt( ISR_pin ) );
          timer2_stop();

          // Aggregate statistics.
          v_pos_a_mean_mean = get_avg( v_pos_a_mean, DATA_STATISTICS_ARRAY_SIZE );
          v_pos_b_mean_mean = get_avg( v_pos_b_mean, DATA_STATISTICS_ARRAY_SIZE );
          v_neg_a_mean_mean = get_avg( v_neg_a_mean, DATA_STATISTICS_ARRAY_SIZE );
          v_neg_b_mean_mean = get_avg( v_neg_b_mean, DATA_STATISTICS_ARRAY_SIZE );
          v_pos_a_std_dev2_mean = get_avg( v_pos_a_std_dev2, DATA_STATISTICS_ARRAY_SIZE );
          v_pos_b_std_dev2_mean = get_avg( v_pos_b_std_dev2, DATA_STATISTICS_ARRAY_SIZE );
          v_neg_a_std_dev2_mean = get_avg( v_neg_a_std_dev2, DATA_STATISTICS_ARRAY_SIZE );
          v_neg_b_std_dev2_mean = get_avg( v_neg_b_std_dev2, DATA_STATISTICS_ARRAY_SIZE );

          emf_mV = get_emf_mV(
            get_mV_from_adc( v_pos_a_mean_mean, v_pos_a_mean_mean ),
            get_mV_from_adc( v_neg_a_mean_mean, v_pos_a_mean_mean ),
            get_mV_from_adc( v_pos_b_mean_mean, v_pos_a_mean_mean ),
            get_mV_from_adc( v_neg_b_mean_mean, v_pos_a_mean_mean ) );

          rpm = get_rpm( emf_mV );

          current_mA = get_current_mA (
            get_mV_from_adc( v_pos_b_mean_mean, v_pos_a_mean_mean ),
            get_mV_from_adc( v_neg_b_mean_mean, v_pos_a_mean_mean ),
            emf_mV,
            motor_R_mOhm );

          torque = get_torque( current_mA );

          waveform_counter = 0;
          waveform_counter_skipped = 0;
          
          data_ready = false;
          is_break_time = false;
          timer2_start(); // does not enable timer ISR, external ISR controls that
          attachInterrupt( digitalPinToInterrupt( ISR_pin ), external_ISR, FALLING );
        }
      }

      if ( millis() > timestamp )  // update user interface(s)
      {
        sprintf( str0, "emf:%4d I:%4d", emf_mV, current_mA );
        sprintf( str1, "rpm:%4d t:%5d", rpm, torque );
        update_lcd( str0, str1 );
        update_serial( str0, str1 );
        timestamp = millis() + user_update_rate; // reset update timer
      }
    } // end while(MIN<duty<MAX) loop
    ramp_dir *= -1; // reverse direction
    duty_cycle += ramp_dir;
  }
}


// Electrical formulas
/* emf is the differences in stable voltages measured across motor terminals
 * while being driven and while not being driven. In a perfect world the
 * source voltage would not change. The world is far from perfect. Should
 * use get_mV_from_adc to get the inputs (motor_pos_1st_mV, etc.).
 */
int get_emf_mV ( int motor_pos_1st_mV, int motor_neg_1st_mV, int motor_pos_2nd_mV, int motor_neg_2nd_mV )
{
  if ( motor_pos_1st_mV < 0 || motor_pos_1st_mV > 9999
    || motor_neg_1st_mV < 0 || motor_neg_1st_mV > 9999
    || motor_pos_2nd_mV < 0 || motor_pos_2nd_mV > 9999
    || motor_neg_2nd_mV < 0 || motor_neg_2nd_mV > 9999 )
  {
    return -1; // error, out of bounds
  }
  
  // Calculation order changed to avoid negative numbers, in case this is changed to use unsigned
  return ( motor_pos_1st_mV + motor_neg_2nd_mV ) - motor_neg_1st_mV - motor_pos_2nd_mV;
}

/* Current is the difference in stable voltages measured across motor terminals
 * while being driven less the emf, over the motor coil resistance. Coil
 * resistance needs to have been measured beforehand, prefereably with an
 * accurate current source and voltmeter (4-point probe, for example), as
 * multimeters have accuracy issues when taking small resistance measurements.
 */
int get_current_mA ( int motor_pos_2nd_mV, int motor_neg_2nd_mV, int emf_mV, int motor_resistance_mOhm )
{
  if ( motor_pos_2nd_mV < 0 || motor_pos_2nd_mV > 9999
    || motor_neg_2nd_mV < 0 || motor_neg_2nd_mV > 9999
    || emf_mV < 0 || emf_mV > 9999
    || motor_resistance_mOhm < 1 )  // it better not be over 32 Ohm (>2^15 mOhm), but not checking
  {
    return -1; // error, out of bounds
  }

  return ( 1000 * ( motor_pos_2nd_mV - motor_neg_2nd_mV - emf_mV ) ) / motor_resistance_mOhm;
}


// Returns the voltage meassured by an ADC pin in [mV]
int get_mV_from_adc( int measurement, int source_voltage_mV )
{
  if ( measurement < 0 || measurement > 1024
    || source_voltage_mV < 0 || source_voltage_mV > 9999 )
  {
    return -1;  // error, out of bounds
  }
  
  // Do calculations at maximum resolution without overrunning
  return (int)( (long)( measurement * source_voltage_mV )/1024 );
}


// Updates LCD
void update_lcd( char * str_row0, char * str_row1 ) // make these char arrays instead of String, later
{
  lcd.clear();
  lcd.print( str_row0 );
  lcd.setCursor( 0,1 );
  lcd.print( str_row1 );
}


// Prints LCD info to serial monitor
void update_serial( String str_row0, String str_row1 ) // make these char arrays instead of String, later
{
  Serial.println();
  Serial.print( "\t\t" );
  Serial.println( str_row0 );
  Serial.print( "\t\t" );
  Serial.println( str_row1 );
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
  if ( timer2_counter >= timer2_counter_limit && !is_break_time )
  {
    digitalWrite( output_pin_B, HIGH );

    if ( next_read_state == 0 )
    {
      read_adc( v_pos_a, DATA_ARRAY_SIZE, motor_pos_pin );
      
      timer2_counter_limit = 3;   // read next at 3/4 PWM period
    }
    else if ( next_read_state == 1 )
    {
      read_adc( v_pos_b, DATA_ARRAY_SIZE, motor_pos_pin );
      
      timer2_counter_limit = 1; // Read next at 1/4 PWM period (1 OVF).
      timer2_counter = 0; // Restart overflow counter.
      timer2_disable();   // Let external ISR trigger this next to re-synchronize.

      // Change adc pins for next read state and throw out 1st reading (change is noisy).
      analogRead( motor_neg_pin );
    }
    else if ( next_read_state == 2 )
    {
      read_adc( v_neg_a, DATA_ARRAY_SIZE, motor_neg_pin );
      
      timer2_counter_limit = 3;   // read next at 3/4 PWM period
    }
    else if ( next_read_state == 3 )
    {
      read_adc( v_neg_b, DATA_ARRAY_SIZE, motor_neg_pin );
      
      timer2_counter_limit = 1; // Read next at 1/4 PWM period (1 OVF).
      timer2_counter = 0; // Restart overflow counter.
      timer2_disable();   // Let external ISR trigger this next to re-synchronize.

      // Change adc pins for next read state and throw out 1st reading (change is noisy).
      analogRead( motor_pos_pin );
    }

    // Increment read state, restart and set flag if finished.
    if ( ++next_read_state > 3 )
    {
      next_read_state = 0;
      data_ready = true;
    }
    
    digitalWrite( output_pin_B, LOW );
  }
  digitalWrite( output_pin_A, LOW );
}


// Helper for ISR( TIMER2_OVF_vect ), so I don't have to fight with array pointers.
void read_adc ( volatile unsigned int data_array[], int data_array_size, int adc_pin )
{
  int n;
  for ( n=0; n<data_array_size; n++)
  {
    data_array[n] = analogRead( adc_pin );
  }
}


// Falling PWM signal triggers this, which starts Timer2 for synchronized ADC measurements.
void external_ISR()
{
  digitalWrite( output_pin_extern_ISR, HIGH );

  /* The below will continuously reset (and enable) Timer2 unless the timer
   * expires before the next external ISR, before this runs again.
   */
  if ( !data_ready && !is_break_time )  // need new data
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


// Returns average of (int)s
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


// Returns square of standard deviation of (int)s
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


// Returns motor speed in [rpm]
int get_rpm( int emf_mV )
{
  return (int)( (long)( emf_mV * ( MOTOR_RPM2 - MOTOR_RPM1 ))/( MOTOR_EMF2 - MOTOR_EMF1 ) );
}


// Returns torque in [mN*cm]
int get_torque( int current )
{
  return (int)( (long)( ( current * MOTOR_TORQUE ) ) / MOTOR_CURRENT );
}
