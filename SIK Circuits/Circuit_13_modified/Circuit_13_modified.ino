
const int relayPin = 2;	    // use this pin to drive the transistor
const int timeDelayMin = 1, timeDelayMax = 250; // min and max delays for testing

void setup() {
  pinMode(relayPin, OUTPUT);  // set pin as an output
  Serial.begin(9600);
}

void loop() {
  serialDelay();
//  showDelays();
} 

// Cycles through various delays for display and testing
void showDelays() {
  int t;
  for (t = pow(2,9); t > 1; t/=2) {
    
    Serial.print("Delay set to ");
    Serial.println(t);
    Serial.println();
    
    cycleRelayOnOff(relayPin, t);
    delay(500);
    cycleRelayOffOn(relayPin, t);
    delay(500);
  }
}


// Set the relay delay time for testing
void serialDelay() {
  int tdelay;
  
  Serial.println("Type a delay (1ms-1000ms) into the box above,");
  Serial.println("then click [send] or press [return].");
  Serial.println();
  
  while(true) {
    while (Serial.available() > 0) {
      tdelay = Serial.parseInt();
      tdelay = constrain(tdelay,1,1000);
      
      Serial.print("Delay set to ");
      Serial.println(tdelay);
      Serial.println();
    }
    
    cycleRelayOnOff(relayPin, tdelay);
  }
}

// Turn relay on and off with specific delay
void cycleRelayOnOff(int pin, int delaytime) {
  digitalWrite(pin, HIGH);
  delay(delaytime);
  digitalWrite(pin, LOW);
  delay(delaytime);
}

// Turn relay off and on with specific delay
void cycleRelayOffOn(int pin, int delaytime) {
  digitalWrite(pin, LOW);
  delay(delaytime);
  digitalWrite(pin, HIGH);
  delay(delaytime);
}
