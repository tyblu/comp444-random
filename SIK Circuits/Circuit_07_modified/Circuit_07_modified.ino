const int temperaturePin = 0;
const int ledPins[] = {9,10,11};
float voltage;

void setup(){
  Serial.begin(9600);
  
  int i;
  for (i = 0; i < 3; i++) {
    pinMode(ledPins[i], OUTPUT);
  }
}

void loop() {
  int ledPWM[3] = {0}, i;
  float temp;
  
  voltage = getVoltage(temperaturePin);
  temp = ( voltage - 0.5 ) * 100.0+5;
  
  ledPWM[0] = (10 - temp)*255/10+255;
  if ( temp<20 ) { ledPWM[1] = (temp - 10)*255/10; }
  else { ledPWM[1] = (20 - temp)*255/10+255; }
  ledPWM[2] = (temp - 25)*255/15;
  
  for (i = 0; i < 3; i++) {
    if ( ledPWM[i] < 255/5 ) { ledPWM[i] = 0; }
    if ( ledPWM[i] > 255 ) { ledPWM[i] = 255; }
    analogWrite( ledPins[i], ledPWM[i] );
  }

  printStuff();
  delay(1000);
}

float getVoltage(int pin) {
  return (analogRead(pin) * 0.0048828125);
}

void printStuff() {
  float degreesC;
  degreesC = (voltage - 0.5) * 100.0+5;
  
  Serial.print("voltage: ");
  Serial.print(voltage);
  Serial.print("  deg C: ");
  Serial.print(degreesC);
  Serial.println();
}
