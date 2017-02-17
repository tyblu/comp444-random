const int RED_PIN = 9;
const int GREEN_PIN = 10;
const int BLUE_PIN = 11;

void setup() {
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
}

void loop() {
  float x, xR, xG, xB, x2sum;
  
  for (x = 0; x < 3*5*7*3142; x++) {
    
    xR = 255*abs(sin(2*x/1000));
    xG = 255*abs(sin(4*x/1000));
    xB = 255*abs(sin(6*x/1000));
    
    // normalize intensities
    x2sum = pow(xR,2)+pow(xG,2)+pow(xB,2);
    if ( x2sum < 0.05 ) { xR = 0; xG = 0; xB = 0; } // get rid of div/zero problems
    else {
      xR = 255*sqrt(pow(xR,2)/x2sum);
      xG = 255*sqrt(pow(xG,2)/x2sum);
      xB = 255*sqrt(pow(xB,2)/x2sum);
    }
    //showRGB( 0, 0, 255 ); // testing
    showRGB( int(xR), int(xG), int(xB) );
    delay(1);
  }
  
  // blink between sets
  showRGB(0,0,0); delay(500);
  showRGB(255,0,0); delay(100); showRGB(0,255,0); delay(100); showRGB(0,0,255); delay(100);
  showRGB(0,0,0); delay(500);
  
  for (x = 0; x < 3*5*7*3142; x++) {
    
    xR = 255*abs(sin(2*x/1000));
    xG = 255*abs(sin(4*x/1000));
    xB = 255*abs(sin(6*x/1000));
    
    showRGB( int(xR), int(xG), int(xB) );
    delay(1);
  }
  
  // blink between sets
  showRGB(0,0,0); delay(500);
  showRGB(255,0,0); delay(100); showRGB(0,255,0); delay(100); showRGB(0,0,255); delay(100);
  showRGB(0,0,0); delay(500);
}

void showRGB(int redIntensity, int greenIntensity, int blueIntensity) {
  // check inputs, must be 0-255
  if ( redIntensity < 0 || greenIntensity < 0 || blueIntensity < 0 ) return;
  if ( redIntensity > 255 || greenIntensity > 255 || blueIntensity > 255 ) return;
  
  analogWrite(RED_PIN, redIntensity);
  analogWrite(BLUE_PIN, blueIntensity);
  analogWrite(GREEN_PIN, greenIntensity);
}
