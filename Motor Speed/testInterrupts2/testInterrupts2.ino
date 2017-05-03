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
 * Version     1.0
 * 
 * References: http://gammon.com.au/interrupts
 *             https://www.programiz.com/cpp-programming/examples/standard-deviation
 */


 const int motor_pin = 9; // PWM driving NPN to control motor
 const int motor_neg_pin = A1;  // to motor -ve and shunt top
 const int ISR_pin = 2;  // external interrupt pin (#3 occupied by LCD)

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
#define DATA_ARRAY_SIZE 16
#define DATA_ARRAY_WINDOW_SIZE 3
#define DATA_ARRAY_SUBWINDOW_SIZE 10
volatile unsigned int vdata[DATA_ARRAY_SIZE] = {0};  // for use in ISR
volatile unsigned int waveform_counter = 0;


void setup() {
  pinMode( motor_pin, OUTPUT );
  
  ADCSRA &= ~prescale_128;    // remove bits set by Arduino library
  ADCSRA |= prescale_016;     // set our own prescaler

  Serial.begin(9600);
  Serial.println("ISR Testing, 1MHz ADC");
  Serial.println("v1.12-20170502\n");
  delay(1500);

  attachInterrupt( digitalPinToInterrupt( ISR_pin ), measure_waveform, RISING ); // enable ISR after stabilization
}

void loop() {

//  Serial.println("Program started");

  analogWrite( motor_pin, 153 );

//  Serial.println("Motor spun up");

//  unsigned long timestamp_write_now = millis() + 1000;
//  if ( millis() > timestamp_write_now ) {
//    Serial.print( "Bytes free in stack: " );
//    Serial.println( freeRam() );
//    timestamp_write_now = millis() + 5000;
//    delay(500);
//  }

  if ( ( waveform_counter+1 ) % 10 == 0 || waveform_counter == 1) {
    detachInterrupt( digitalPinToInterrupt( ISR_pin ) );
    Serial.print("Let's take a breather! Whew, already ");
    Serial.print( waveform_counter );
    Serial.println(" waveforms into it!");
    Serial.println("Let's take a closer look at the last waveform...");

    int n;
    for ( n=0; n<DATA_ARRAY_SIZE; n++ ) {
      Serial.println(vdata[n]);
    }
    
    Serial.print("\nSend any character to continue...");
    while( !Serial.findUntil( "Go", '!' ) ) { delay(100); }
    Serial.println("... Well, back at it!");
    delay(1000);
    
    attachInterrupt( digitalPinToInterrupt( ISR_pin ), measure_waveform, RISING );
  }
}


// ISR for PWM 
void measure_waveform() {
//  Serial.print("I\'m in! Waveform #"); Serial.println(waveform_counter);                     // debugging
  int n;
  for ( n=0; n<DATA_ARRAY_SIZE; n++ ) {
    vdata[n] = analogRead( motor_neg_pin );
  }
  waveform_counter++;
//  Serial.print("I\'m out! Next waveform is #"); Serial.println(waveform_counter);                     // debugging
}


// reports space between the heap and the stack
int freeRam () // https://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
