/********************************************************************
 * Based on SparkFun Inventor's Kit example #12: SPINNING A MOTOR
 * 
 * Set motor speed from serial monitor.
 *******************************************************************/

const int motorPin = 9;

void setup() {
  pinMode(motorPin, OUTPUT);  // set up the pin as an OUTPUT
  Serial.begin(9600);         // initialize Serial communications
}

void loop() {
  int speed = 0;

  Serial.print("Type a speed (0-255) and press [return]: ");

  while(true) {
    while (Serial.available() > 0) {
      
      speed = Serial.parseInt();  // parseInt() reads 1st int
      speed = constrain(speed, 0, 255); // constrains between 0 and 255
      
      Serial.print("Setting speed to ");
      Serial.print(speed);
      
      analogWrite(motorPin, speed);
      
      int n; for (n=0;n<3;n++) { delay(100); Serial.print("."); }
      Serial.println(" Speed set.\n");
      
      Serial.print("Type a speed (0-255) and press [return]: ");
    }
  }
}
