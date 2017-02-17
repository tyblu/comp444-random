int sensorPin = 0;    // The potentiometer is connected to analog pin 0
int ledPin = 13;      // The LED is connected to digital pin 13


void setup()
{
  pinMode(ledPin, OUTPUT);
}


void loop()
{
  int sensorValue;
  sensorValue = analogRead(sensorPin);    

  int freq, tdelay;
  
  freq = 30*(1024-sensorValue)/1024;   // scale frequency with voltage
  
  if ( freq < 1 )        // div by 0 errors
    tdelay = 1000/(2*0.5);      // Humans can't really tell small differences below 1Hz, so just make it 0.5 Hz
  else if ( freq > 29 )
    tdelay = 1000/(2*30);      // Humans also can't really see over 30Hz, so just make it 30 Hz
  else
    tdelay = 1000/(2*freq);

  digitalWrite(ledPin, HIGH);     // Turn the LED on
  delay(tdelay);             // Pause for sensorValue milliseconds
  digitalWrite(ledPin, LOW);      // Turn the LED off
  delay(tdelay);             // Pause for sensorValue milliseconds
}
