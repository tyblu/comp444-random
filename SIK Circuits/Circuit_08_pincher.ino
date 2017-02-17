#include <Servo.h>

Servo servo1;

void setup() {
  servo1.attach(9);
}

void loop() {
  int position;
  int posMax = 180-8; int posMin = 60; // posMax is closed, posMin is open
  
  for ( position = posMin; position < posMax; position +=1 ){
    servo1.write(position);
    delay(20);
  }
  
  servo1.write(posMax); delay(500);
  
  for ( position = posMax; position > posMin; position -= 1) {
    servo1.write(position);
    delay(20);
  }
  
  servo1.write(posMin); delay(100);
}
