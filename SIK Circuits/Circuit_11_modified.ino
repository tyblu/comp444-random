const int buzzerPin = 9;

// The tempo is how fast to play the song. To make the song play faster, decrease this value.
const int tempo = 150;


void setup() 
{
  pinMode(buzzerPin, OUTPUT);
}


void loop() 
{
  int i;
  for (i = 1; i < 88; i++) {
    tone(buzzerPin, (int)( 440 * pow( (float)2, (float)(i-49)/(float)12 ) ), tempo*4);
    delay(tempo*4);       // wait for tone to finish
  }
  
  delay(1000);
  
  //while(true){}
}
