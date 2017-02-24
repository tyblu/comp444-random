const int motorPin = 9;
const int speedLimits[] = {34, 255, 255};  // Motor stall/min speed, start speed, max speed
const int timeStart = 15;  // Time to get motor moving from rest with start speed voltage

void setup() {
  pinMode(motorPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  motorAcceleration();
//     serialSpeed();
}

// Jump-starts the motor and sets it at any speed. 
void motorGo(int speedGoal) {
  
  if (speedGoal <= speedLimits[0]) {
    speedGoal = 0;
  }
  else {
    speedGoal = constrain(speedGoal, speedLimits[0], speedLimits[2]);
    analogWrite(motorPin, speedLimits[1]);  // kickstart motor on
    delay(timeStart);
  }
  
  analogWrite(motorPin, speedGoal);      // set speed
  
  Serial.print("--> Speed set to ");     // write about it
  Serial.println(speedGoal);
}

// This function slowly accelerates the motor to full speed, then back down to zero.
void motorAcceleration() {
  int speed;
  int delayTime = 20; // milliseconds between each speed step
  
  // accelerate the motor
  for(speed = 0; speed <= 255; speed++) {
    motorGo(speed);	// set the new speed
    delay(delayTime);   // delay between speed steps
  }
  
  // decelerate the motor
  for(speed = 255; speed >= 0; speed--) {
    motorGo(speed);	// set the new speed
    delay(delayTime);   // delay between speed steps
  }
}

// This function will let you type a speed into the serial monitor window. Open the serial
// monitor using the magnifying-glass icon at the top right of the Arduino window. Then
// type your desired speed into the small text entry bar at the top of the window and click
// "Send" or press return. The motor will then operate at that speed. (Valid range 0-255.)
void serialSpeed() {
  int speed;
  
  Serial.println("Type a speed (0-255) into the box above,");
  Serial.println("then click [send] or press [return]");
  Serial.println();  // Print a blank line

  while(true) {  // loop forever
    // First we check to see if incoming data is available:
    while (Serial.available() > 0) {
      // If it is, we'll use parseInt() to pull out any numbers:
      speed = Serial.parseInt();
      speed = constrain(speed, 0, 255);
      
      // We'll print out a message to let you know that the number was received:
      Serial.print("Setting speed to ");
      Serial.println(speed);
  
      motorGo(speed);  // And finally, we'll set the speed of the motor!
    }
  }
}


