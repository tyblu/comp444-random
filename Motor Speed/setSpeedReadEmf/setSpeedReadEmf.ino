/********************************************************************
 * Based on SparkFun Inventor's Kit example #12: SPINNING A MOTOR,
 * example #7: TEMPERATURE SENSOR, and example #15: LIQUID CRYSTAL 
 * DISPLAY (LCD)
 * 
 * Set motor speed from serial monitor. Put motor driving transistor
 * in high impedance (motorPin set to LOW) for a short time, 
 * measure the back-emf during this pause, then continue driving the
 * motor. Measured value, EMF voltage and motor speed are output to 
 * an LCD.
 *******************************************************************/

#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

const int motorPin = 9; // conected to transistor base to drive motor
const int emfPin = A0;  // analog input related to motor back-emf

const int delayTimeStabilize = 3; // time for emf to stabilize [ms]
const int delayTime = 10 - delayTimeStabilize; // pause time [ms]
const int runTime = 1000 - delayTime; // run time [ms]

// linear relationship between emf and speed, rpm = emf * k1 + k0
// measure low rpm and voltage to get rpm0 and volt0
// measure high rpm and voltage to get rpm1, volt1
// k1 = ( rpm1 - rpm0 ) / ( volt1 - volt0 )
// k0 = rpm0 - volt0 * k1, or k0 = rpm1 - volt1 * k1
const float emfToRpmK1 = ( 6600 - 1500 ) / ( 3.2 - 1.0 );
const float emfToRpmK0 = 6600 - 3.2 * emfToRpmK1;


void setup() {
  pinMode(motorPin, OUTPUT);  // set up the pin as an OUTPUT

  lcd.begin(16, 2); //Initialize the 16x2 LCD
  lcd.clear();  // clear any old data displayed on the LCD
  lcd.print("Hello, COMP444!");
  lcd.setCursor(0,1);
  lcd.print("emf to RPM v1.0");
  delay(500);
}

void loop() {

  // initialize variables once
  float vAnalog, vEmf;
  int rpm = 0, rpmOld = 0, rpmChange = 0;
  int rpmMin = 2*6600, rpmMax = 0, rpmStart = 2*6600;

  while(true) {

    // ramp speed up and measure at each of 10 steps
    int n;
    for ( n = 0 ; n < 10 ; n++ ) {
      // run motor
      motorRunAndWait(motorPin, n * 255 / 10, runTime);
  
      // pause, measure back-emf, un-pause
      vAnalog = pauseAndMeasure(motorPin, emfPin, delayTimeStabilize, delayTime, n*255/10);

      // get emf and rpm
      vEmf = 5.0 - vAnalog; // emf = supply voltage - measurement
      rpmOld = rpm;   // save last rpm value before overwriting
      rpm = vEmf * emfToRpmK1 + emfToRpmK0 + 0.5; // get speed
      rpmLogic( rpm, rpmOld, &rpmMin, &rpmMax, &rpmStart ); // set rpmMin, rpmMax, rpmStart

      // print to LCD
      printAllTheThings( vEmf, rpm, rpmOld, rpmMin, rpmMax, rpmStart );
    }

    // ramp speed down and measure at each of 10 steps
    for ( ; n ; n-- ) {
      // run motor
      motorRunAndWait(motorPin, n * 255 / 10, runTime);
  
      // pause, measure back-emf, un-pause
      vAnalog = pauseAndMeasure(motorPin, emfPin, delayTimeStabilize, delayTime, n*255/10);

      // get emf and rpm
      vEmf = 5.0 - vAnalog; // emf = supply voltage - measurement
      rpmOld = rpm;   // save last rpm value before overwriting
      rpm = vEmf * emfToRpmK1 + emfToRpmK0 + 0.5; // get speed
      rpmLogic( rpm, rpmOld, &rpmMin, &rpmMax, &rpmStart ); // set rpmMin, rpmMax, rpmStart

      // print to LCD
      printAllTheThings( vEmf, rpm, rpmOld, rpmMin, rpmMax, rpmStart );
    }
  }
}

// sets motor speed to a certain value, then waits a bit
void motorRunAndWait( int mPin, int mSpeed, int tDelay ) {
  analogWrite(mPin, mSpeed);
  delay(tDelay);
}

// pauses motor driver, measures emf-related value, restarts motor
float pauseAndMeasure( int mPin, int mSpeed, int emfPin, int tStab, int tDelay ) {
  float vAnalog = 0.0;

  analogWrite(mPin, LOW); // cut current to motor
  delay(tStab); // wait for emf to stabilize

  int t; // take measurement every 1ms and average them all
  for ( t=0 ; t <= tDelay ; t++ ) {
    vAnalog += analogRead(emfPin) / tDelay;
    delay(1);
  }
  
  analogWrite(mPin, mSpeed);  // continue driving motor
  
  return vAnalog;
}

// logic to set rpmMin, rpmMax, rpmStart
void rpmLogic( int rpm, int rpmOld, int* rpmMin, int* rpmMax, int* rpmStart ) {

  int rpmChange;  // by how much did the speed change?
  rpmChange = rpm - rpmOld;

  if ( rpmOld < *rpmStart && rpmChange > 100 ) {  // just started, lower start rpm
    *rpmStart = rpmOld;
  }
  if ( rpm > 100 && rpm < *rpmMin ) {             // running, lower min rpm
    *rpmMin = rpm;
  }
  if ( rpm > *rpmMax ) {                          // higher max rpm
    *rpmMax = rpm;
  }
}


// print to LCD
void printAllTheThings( float emf, int rpm, int rpmOld, int rpmMin, int rpmMax, int rpmStart ) {

  int rpmChange;
  rpmChange = rpm - rpmOld;
  
  lcd.clear();
  
  lcd.setCursor(0,0);           // 1st line [0000rpm 0.00000V]
  lcd.print(rpm);
  lcd.print("rpm, ");
  lcd.print(emf,5);
  lcd.print("V");
  
  lcd.setCursor(0,1);           // 2nd line [0.0,0.0,0.0,+0.0]
  printInKRpm( rpmMin, 1 );  // (min, max, start, change rpm/1000)
  printInKRpm( rpmMax, 1 );
  printInKRpm( rpmStart, 1 );
  rpmChange > 0 ? lcd.print("+") : lcd.print("-");
  printInKRpm( rpmChange, 0 );
}

// helper for printAllTheThings()
void printInKRpm( int num , bool printComma) {
  String sNum = String( num );
  lcd.print( sNum[0] );
  lcd.print(".");
  lcd.print( sNum[1] );
  if ( printComma ) { lcd.print(","); }
}
