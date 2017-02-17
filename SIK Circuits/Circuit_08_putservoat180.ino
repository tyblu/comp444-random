#include <Servo.h>

Servo servo1;

void setup() {
  servo1.attach(9);
}

void loop() {
  int position;
  
  servo1.write(180);
}
