const int button1Pin = 2;  // pushbutton 1 pin
const int button2Pin = 3;  // pushbutton 2 pin
const int ledPin =  13;    // LED pin

int count = 0;

void setup() {
  pinMode(button1Pin, INPUT);
  pinMode(button2Pin, INPUT);

  pinMode(ledPin, OUTPUT);  

}

void loop() {
  int button1State, button2State;  // variables to hold the pushbutton states
  int i;

  button1State = digitalRead(button1Pin);
  button2State = digitalRead(button2Pin);
  
  if ( (button1State == LOW) && (button2State == LOW) ) {
    digitalWrite(ledPin, HIGH);
    count++;
    delay(100);
    if ( count >= 10 ) {
      for ( i = 0; i < 10; i++ ){
        digitalWrite(ledPin, HIGH); delay(100);
        digitalWrite(ledPin, LOW); delay(100);
      }
    }
  }
  else {
    digitalWrite(ledPin, LOW);
    count = 0;
  }
}
