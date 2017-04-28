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

const int delayTimeStabilize = 1; // time for emf to stabilize [ms]
const int delayTime = 5 - delayTimeStabilize; // pause time [ms]
const int runTime = 500 - delayTime; // run time [ms]

// linear relationship between emf and speed, rpm = emf * k
// measure low rpm and voltage to get rpm0 and volt0
// measure high rpm and voltage to get rpm1, volt1
// k = ( rpm1 - rpm0 ) / ( volt1 - volt0 )
const float emfToRpmK = ( 6120 - 2400 ) / ( 3.0 - 1.2 ); // about 2066rpm/V

struct rpmStruct {
  int current;
  int old;
  int minimum;
  int maximum;
  int start;
};

void setup() {
  pinMode(motorPin, OUTPUT);  // set up the pin as an OUTPUT

  lcd.begin(16, 2); //Initialize the 16x2 LCD
  lcd.clear();  // clear any old data displayed on the LCD
  lcd.print("Hello, COMP444!");
  lcd.setCursor(0,1);
  lcd.print("emf to RPM v1.0");
  delay(1500);
}

void loop() {

  // initialize variables once
  struct rpmStruct rpm = {
    .current = 0,
    .old = 0,
    .minimum = 9999,
    .maximum = 0,
    .start = 9999,
  };
  char lcdText[2][16] = { { ' ' } }; // 16x2 spaces
  float vAnalog = 0.0, vEmf = 0.0;

  int n = 0, rampDir = 1;
  while ( true ) {
    
    // ramp speed up and measure at each of 10 steps
    while ( n != 11 && n != -1 ) {
      
      motorRunAndWait(motorPin, n * 255 / 10, runTime);  // run motor
      
      // pause, measure back-emf, un-pause
      vAnalog = pauseAndMeasure( motorPin, n*255/10, emfPin, delayTimeStabilize, delayTime );
      
      // get emf and rpm
      vEmf = 5.0 - vAnalog; // emf = supply voltage - measurement
      
      rpm.old = rpm.current;   // save last rpm value before overwriting
      
      rpm.current = vEmf * emfToRpmK; // get speed
      
      rpmLogic( rpm ); // set rpmMin, rpmMax, rpmStart
      
      printAllTheThings( lcdText, vEmf, rpm );  // print to LCD
      
      n += rampDir;
    }
    rampDir *= -1;  // change ramp direction
    n += rampDir;   // bump index so loop can go again
  }

  // wait and tell user that program is restarting in 5-4-3...
//  printRestartMsgs( 6000 );
  lcd.clear(); lcd.print("RESTARTING"); // debug line
}

// sets motor speed to a certain value, then waits a bit
void motorRunAndWait( int mPin, int mSpeed, int tDelay ) {
  analogWrite(mPin, mSpeed);
  delay(tDelay);
}

// pauses motor driver, measures emf-related value, restarts motor
float pauseAndMeasure( int mPin, int mSpeed, int emfPin, int tStab, int tDelay ) {
  int vMeasured;
  float vAnalog;

  analogWrite(mPin, LOW); // cut current to motor
  delay(tStab); // wait for emf to stabilize

  int t; // take measurement every 1ms and average them all
  for ( t=0 ; t <= tDelay ; t++ ) {
//    lcd.clear(); lcd.setCursor(0,0); lcd.print(analogRead(emfPin)); delay(1000);// debugging
    vMeasured = analogRead(emfPin);
    vAnalog = 5 * vMeasured / 1024;
    delay(1);
  }
//  char debug[16]; sprintf( debug, "n:%dvA0:%f5%+d", n, vAnalog, rampDir );  // debugging
//  lcd.setCursor(0,0); lcd.print(debug); delay(1000);                        // debugging
  analogWrite(mPin, mSpeed);  // continue driving motor
  
  return vAnalog;
}

// logic to set rpmMin, rpmMax, rpmStart
void rpmLogic( struct rpmStruct &rpm ){
//void rpmLogic( int &rpm, int &rpmOld, int &rpmMin, int &rpmMax, int &rpmStart ) {

  int rpmChange;  // by how much did the speed change?
  rpmChange = rpm.current - rpm.old;

  if ( rpm.old < rpm.start && rpmChange > 100 ) {  // just started, lower start rpm
    rpm.start = rpm.current;
  }
  if ( rpm.current > 100 && rpm.current < rpm.minimum ) {             // running, lower min rpm
    rpm.minimum = rpm.current;
  }
  if ( rpm.current > rpm.maximum ) {                          // higher max rpm
    rpm.maximum = rpm.current;
  }
}


// print to LCD
void printAllTheThings( char (&lcdText)[2][16], float &emf, struct rpmStruct &rpm ) {
//void printAllTheThings( char (&str0)[16], 
//                        char (&str1)[16], 
//                        float &emf, 
//                        int &rpm, 
//                        int &rpmOld, 
//                        int &rpmMin, 
//                        int &rpmMax, 
//                        int &rpmStart ) {

  char sTemp[] = "                ";  // initialize to maximum size

  int rpmChange;
  rpmChange = rpm.current - rpm.old;
  
  lcd.clear();
  
  lcd.setCursor(0,0);           // 1st line [0000rpm 0.00000V]
  sprintf( lcdText[0], "%5drpm, %5fV", rpm.current, emf );
//  lcd.print( str0 );
//  sTemp = intToChar( sTemp, rpm )
//  insertSubstrAtIndex( str0, 16, sTemp, 4, 0 );
//  sTemp = "rpm,";
//  insertSubstrAtIndex( str0, 16, sTemp, 4 );
//  sTemp = String( emf, 4 );
//  insertSubstrAtIndex( str0, 16, sTemp, 6), 9 );
//  sTemp = "V";
//  insertSubstrAtIndex( str0, 16, sTemp, 1, 15 );
////  ( char* str, int strLen, char* subStr, int subStrLen, int index )
////  lcd.print(rpm);
////  lcd.print("rpm,");
////  lcd.print(emf,4);
////  lcd.print("V");

  sprintf( lcdText[1], "%1d.%1d,%1d.%1d,%1d.%1d,%+1d.%1d",
    intMSBs( rpm.minimum, 1 ),
    intMSBs( rpm.minimum - intMSBs( rpm.minimum, 1 ), 1 ),
    intMSBs( rpm.maximum, 1 ),
    intMSBs( rpm.maximum - intMSBs( rpm.maximum, 1 ), 1 ),
    intMSBs( rpm.start, 1 ),
    intMSBs( rpm.start - intMSBs( rpm.start, 1 ), 1 ),
    intMSBs( rpmChange, 1 ),
    intMSBs( rpmChange - intMSBs( rpmChange, 1 ), 1 )
  );
//  lcd.setCursor(0,1);           // 2nd line [0.0,0.0,0.0,+0.0]
//  printInKRpm( rpmMin, 1 );  // (min, max, start, change rpm/1000)
//  printInKRpm( rpmMax, 1 );
//  printInKRpm( rpmStart, 1 );
//  rpmChange > 0 ? lcd.print("+") : lcd.print("-");
//  printInKRpm( abs( rpmChange ), 0 );
}

// helper for printAllTheThings()
void printInKRpm( int num , bool printComma) {
  char sNum[15] = {0}; 
  sprintf( sNum, "%d", num );
  lcd.print( sNum[0] );
  lcd.print(".");
  lcd.print( sNum[1] );
  if ( printComma ) { lcd.print(","); }
}

// wait and tell user that program is restarting in 5-4-3...
void printRestartMsgs( int t ) {

  unsigned long tEnd = millis() + (unsigned long)( t ); // deadline

  // scroll old text off screen to the right
  lcd.setCursor(0,0);
  int counter;
  for ( counter = 0 ; counter < 16 ; counter++ ) {
    if ( millis() > tEnd ) { lcd.clear(); lcd.print("ERROR"); return; }
    lcd.scrollDisplayRight();
    delay(50);
  }

  // write 'restart' message, with countdown
  lcd.setCursor(0,0);
  lcd.print("                "); // 16 spaces to clear row
  lcd.rightToLeft();
  lcd.setCursor(15,0);
  int tRemainSec = (int)( tEnd - millis() )/1000; // time left [seconds]
//  printOneLetterAtATime( "Restarting in ", 14, min( 50, ( tRemainSec-2 )*1000/14/2 ) );
  lcd.print( "Restarting in ");   // do this while above line broken
  while ( tRemainSec > 0 ) {
    if ( millis() > tEnd ) { lcd.clear(); lcd.print("ERROR"); return; }
    lcd.print( tRemainSec );
    printDotsFor1Sec( 3 );
    tRemainSec--;
  }

  // write 'restarting now' message
  lcd.setCursor(0,0);
  lcd.leftToRight();
  lcd.print("                "); // 16 spaces to clear row
  lcd.setCursor(0,0);
  lcd.print("Restarting NOW.");
}

//void printOneLetterAtATime( char &str, int strLen, int tdelay ) {
//  int n;
//  for ( n=0 ; n<strLen ; n++ ) {
//    lcd.print( str[n] );
//    delay( tdelay );
//  }
//}

void printDotsFor1Sec( int nDots ){
  int n;
  for ( n=0; n<nDots; n++ ) {
    delay( 1000 / nDots );
    lcd.print(".");
  }
}

void fillStrWithSpacesFromIndex( char (&str)[64], int strLen, int index ) {
  while ( index < strLen ) {
    str[index++] = ' ';
  }
}

void insertSubstrAtIndex( char (&str)[64], int strLen, char (&subStr)[64], int subStrLen, int index ) {
  int i = 0;
  while ( index < strLen && i < subStrLen) {
    str[index++] = subStr[i++];
  }
}

void updateLCD( char (&txt)[2][16] ) {
  lcd.clear();
  lcd.leftToRight();
  lcd.print( txt[0] );
  lcd.setCursor( 1,0 );
  lcd.print( txt[1] );
}

int intMSBs( int num , int sigDigs) {
  while ( num > powInt( 10, sigDigs ) ) {
    num /= 10;
  }
  return num;
}

int powInt( int base, int power) {
  if ( power < 0 ) { return 0; }
  int result = 1;
  while ( power-- > 0 ) {
    result *= base;
  }
  return result;
}

