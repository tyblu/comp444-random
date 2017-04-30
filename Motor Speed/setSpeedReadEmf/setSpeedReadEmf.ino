/********************************************************************
 * Based on SparkFun Inventor's Kit example #12: SPINNING A MOTOR,
 * example #7: TEMPERATURE SENSOR, and example #15: LIQUID CRYSTAL 
 * DISPLAY (LCD)
 * 
 * Calculate EMF while motor is running, no pausing required.
 *******************************************************************/

#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

const int motor_pin = 9; // PWM driving NPN to control motor
const int vA_pin = A0;  // to motor +ve
const int vB_pin = A1;  // to motor -ve and shunt top
const int vC_pin = A2;  // to shunt bottom and collector

struct node_voltages {
  int a[1000];
  int b[1000];
  int c[1000];
  int ab[100];
  int bc[100];
};

const float k_volts_per_adc_bit = 5/1024;     // 5V/2^10
const float R_shunt = 0.5;
const float k_Rcoils_Rshunt_const = 1.95/R_shunt; // Rcoils=1.95, Rshunt=0.5

struct pwm_set_points {
  int target;
  int current;
  int steps;
  int next;
  int target_to_current;
  int target_to_current_abs;
};

void setup() {
  pinMode( motor_pin, OUTPUT );

  lcd.begin(16, 2); //Initialize the 16x2 LCD
  lcd.clear();  // clear any old data displayed on the LCD
  lcd.print("Hello, COMP444!");
  lcd.setCursor(0,1);
  lcd.print("emf to RPM v2.3");
  delay(1500);
}

void loop() {

  lcd.clear();
  lcd.print( "emf: (unknown)" );
  lcd.setCursor( 0,1 );
  lcd.print( "  I: (unknown)" );

  pwm_set_points m_pwm {
    .target                 = 0,
    .current                = 0,
    .steps                  = 0,
    .next                   = 0,
    .target_to_current      = 0,
    .target_to_current_abs  = 0,
  };
    
  node_voltages node;   // holds history (arrays) of node voltages used to calculate emf and current
  int m = 0; // index node_voltages.a,b,c arrays (while loop counter)
  int n = 0; // index node_voltages.ab,bc arrays (m/10)

  float emf[10] = {0};
  float current[10] = {0};

  unsigned long lcd_update_period = 1000000;    // update LCD every 1 sec, in [us]
  unsigned long timestamp = micros() + lcd_update_period;

  while ( true ) {

     m_pwm.target = 153; // 1V-3V rating/5V supply=20-60% duty=51-153pwm

    // ramp motor speed to target
    while ( m_pwm.target != m_pwm.current ) {
      m_pwm.target_to_current = m_pwm.target - m_pwm.current;
      m_pwm.target_to_current_abs = abs( m_pwm.target_to_current );
      m_pwm.steps = ( m_pwm.target_to_current_abs + 10 )/10; // most sig. digit+1
      m_pwm.next = m_pwm.current + m_pwm.target_to_current / m_pwm.steps;
  
      analogWrite( motor_pin, m_pwm.next );
      delay(200); // slow ramp
  
      m_pwm.current = m_pwm.next;
    }

    /* Measurements.
     *  Notes:
     *  - 1st read may be garbage due to adc multiplexer switch noise. Can put
     *  in delay and read again, or do proper code without Arduino libraries.
     *  - Each reading takes about 100us. Might be able to do them more quickly
     *  without Arduino libraries, maybe even faster than consecutively. (Does
     *  the ATmega328p have multiple ADCs, or just a multiplexer?)
     */
    node.a[m] = analogRead( vA_pin );  // 1st read garbage due to adc multiplex noise, delay stabilizes
    node.b[m] = analogRead( vB_pin );
    node.c[m] = analogRead( vC_pin );
    m++;

    if ( m % 10 == 0 ) {  // we have some multiple of and at least 10 measurements

      int j, node_temp[3] = {0};
      for ( j=0; j<10; j++ ) {          // add up last 10 values
        node_temp[0] += node.a[m-j-1];
        node_temp[1] += node.b[m-j-1];
        node_temp[2] += node.c[m-j-1];
      }
      node.ab[n] = ( node_temp[0] - node_temp[1] )/10;  // Vab avg over last 100 values
      node.bc[n] = ( node_temp[1] - node_temp[2] )/10;  // Vbc avg over last 100 values
      emf[n] = ( node.ab[n] - node.bc[n] * k_Rcoils_Rshunt_const ) * k_volts_per_adc_bit;
      current[n] = node.bc[n] / R_shunt;
      n++;
    }

    // spit out emf and current to user every X secs
    if ( micros() > timestamp && n > 0) {      
      lcd.clear();
//      lcd.print( str_temp );                                // broken
//      lcd.print( "emf: " );
//      lcd.print( emf[n-1], 4 );
//      lcd.print( node.a[m-1] );                               // debugging1
      lcd.print( node.ab[n-1] );                              // debugging2
      lcd.print( " " );                                       // debugging1
      lcd.print( node.bc[n-1] );                              // debugging2
//      lcd.print( node.b[m-1] );                               // debugging1
      lcd.setCursor( 0,1 );                                   // debugging1
//      lcd.print( node.c[m-1] );                               // debugging1
      lcd.print( " " );                                       // debugging
      lcd.print( n );                                         // debugging
      lcd.print( ":" );                                       // debugging
      lcd.print( m );                                         // debugging
//      lcd.setCursor( 0,1 );
//      lcd.print( "  I: " );
//      lcd.print( current[n-1], 4 );

      // should probably spit out rpm somewhere around here, too

      timestamp = micros() + lcd_update_period;
    }

    if ( m > 99 ) { m = 0; String aa="9 bottles"; String bb=" "; lcdPrintAll(aa,bb); delay(500); } // end of array, reset index
    if ( n > 9 ) { n = 0; String cc="9 mo bottles"; String dd=" "; lcdPrintAll(cc,dd); delay(500); } // end of array, reset index

  }
}

void lcdPrintAll( String &str0, String &str1 ) {
//void lcdPrintAll( char &str0, char &str1 ) {
  while ( true ) {
    lcd.clear();
    lcd.print( str0 );
    lcd.setCursor( 1,0 );
    lcd.print( str1 );
  }
}

