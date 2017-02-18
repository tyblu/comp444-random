int ledPins[] = {2,3,5,6,9,10,11,13};
int pinIntensity[8] = {0};
int tDelay = 2;

void setup() {
  int index;
  
  for ( index = 0; index <= 7; index++ ) {
    pinMode( ledPins[index], OUTPUT);
  }
}

void loop() {

  // DEBUGGING
  //digitalWrite( ledPins[0], HIGH ); digitalWrite( ledPins[7], HIGH ); delay(1000);
  //pinIntensity[0] = 255; pinIntensity[7] = 255; lightLED(); delay(1000);
  //pinIntensity[0] = 0; pinIntensity[7] = 0; lightLED(); delay(1000);
  
  firstToLast(); // 1st to last, 2 at a time, slow on and slow off, then fade off
  allSlowOnThenOff();
  endsToMiddleThenBack();
  scatter();
}

void firstToLast(){
  int p, pOthers, pinIntensitySum;
  
  pinIntensity[0] = 255; lightLED(); delay(500);    // light 1st LED
  
  for ( p = 1; p <= 6; p++ ){
    while ( pinIntensity[p] < 255 ) {
      pinIntensity[p] += 2;
      for ( pOthers = 0; pOthers <= 7; pOthers++){
        if ( abs(p-pOthers) == 1 && pinIntensity[pOthers] < pinIntensity[p]/3 ) { pinIntensity[pOthers] += 1; }
//        else if ( abs(p-pOthers) == 2 && pinIntensity[pOthers] < pinIntensity[p]/5 ) { pinIntensity[pOthers] += 1; }
        else if ( abs(p-pOthers) == 1 && pinIntensity[pOthers] > pinIntensity[p]/3 ) { pinIntensity[pOthers] -= 1; }
//        else if ( abs(p-pOthers) == 2 && pinIntensity[pOthers] > pinIntensity[p]/5 ) { pinIntensity[pOthers] -= 1; }
        else {
          pinIntensity[pOthers] -= 1;
          if ( pinIntensity[pOthers] < 0 ) { pinIntensity[pOthers] = 0; }
        }
      }
      lightLED(); delay(tDelay);
    }
  }
  
  pinIntensity[7] = 255; lightLED(); delay(500);   // light last LED
  
  do {                                  // dim all LEDs
    pinIntensitySum = 0;
    for ( p = 0; p <= 7; p++ ){
      pinIntensity[p] = abs(pinIntensity[p]);
      pinIntensity[p] -= 1;
      pinIntensitySum += pinIntensity[p];
    }
    lightLED(); delay(tDelay);
  } while ( pinIntensitySum > 0 );
}

void allSlowOnThenOff(){
  int i, intensity;
  for ( intensity = 0; intensity < 255; intensity++ ) {
    for ( i = 0; i <= 7; i++){
      pinIntensity[i] = intensity;
    }
    lightLED(); delay(tDelay*5);
  }
  for ( ; intensity > 0; intensity-- ) {
    for ( i = 0; i <= 7; i++){
      pinIntensity[i] = intensity;
    }
    lightLED(); delay(tDelay*5);
  }
}

void endsToMiddleThenBack(){
  int intensity, i, m, pinIntensitySum;

  for ( i = 0; i < 4; i++ ){
    for ( intensity = 0; intensity < 255; intensity++ ){
      pinIntensity[i] += 1;
      pinIntensity[7-i] += 1;
      for ( m = 0; m <= 7; m++ ){
        if ( m != i && m != (7-i) ) { pinIntensity[m] -= 1; }
      }
      lightLED(); delay(tDelay*2);
    }
  }
  
    for ( i = 3; i > 0; i-- ){
    for ( intensity = 255; intensity > 0; intensity-- ){
      pinIntensity[i] += 1;
      pinIntensity[7-i] += 1;
      for ( m = 0; m <= 7; m++ ){
        if ( m != i && m != (7-i) ) { pinIntensity[m] -= 1; }
      }
      lightLED(); delay(tDelay*2);
    }
  }
  do {                                  // dim all LEDs
    pinIntensitySum = 0;
    for ( i = 0; i <= 7; i++ ){
      pinIntensity[i] = abs(pinIntensity[i]);
      pinIntensity[i] -= 1;
      pinIntensitySum += pinIntensity[i];
    }
    lightLED(); delay(tDelay);
  } while ( pinIntensitySum > 0 );
}

void scatter(){
  int i, intensity, t;
  
  for ( t = 0; t < 500*tDelay; t++) {
    i = int(random(0,7));
    intensity = int(random(0,255));
    if ( intensity < 255/3 ) { intensity = 0; }
    pinIntensity[i] = intensity;
    lightLED(); delay(tDelay);
  }
}

void lightLED() {
  int i;
  for ( i = 0; i <= 7; i++ ){
    if ( i == 0 || i == 7 ) {
      if ( pinIntensity[i] > 3 ) { digitalWrite( ledPins[i], HIGH ); }
      else { digitalWrite( ledPins[i], LOW ); }
    }
    else {
      if ( pinIntensity[i] < 0 ) { pinIntensity[i] = 0; }
      if ( pinIntensity[i] > 255 ) { pinIntensity[i] = 255; }
      analogWrite( ledPins[i], pinIntensity[i] );
    }
  }
}
