#define trigPin 13
#define echoPin 12

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  Serial.begin(9600);
}

void loop() {
  long duration, distance;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
//  distance = ( duration / 2 )/ 29.1;  // fix this value

  Serial.println(duration);
//  Serial.println(distance);

  delay(500);
}
