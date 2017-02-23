const int buzzerPin = 9;
const int tempo = 150;

void setup() 
{
  pinMode(buzzerPin, OUTPUT);
  Serial.begin(9600);
}


void loop() 
{
  int keyNum, freq;
  char str[18];
  
  for (keyNum = 1; keyNum < 88; keyNum++) {
    freq = (int)( 440 * pow( (float)2, (float)(keyNum-49)/(float)12 ) );
    tone(buzzerPin, freq, tempo*10);
    delay(tempo*10);       // wait for tone to finish
    
    sprintf(str, "key %02d  freq %05d", keyNum, freq);
    Serial.println(str);
  }
  
  delay(1000);
  
  while(true){}
}
