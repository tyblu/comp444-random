// include the library code:
#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// make some custom characters:
byte heart[8] = {
  0b00000,
  0b01010,
  0b11111,
  0b11111,
  0b11111,
  0b01110,
  0b00100,
  0b00000
};

byte smiley[8] = {
  0b00000,
  0b00000,
  0b01010,
  0b00000,
  0b00000,
  0b10001,
  0b01110,
  0b00000
};

byte frownie[8] = {
  0b00000,
  0b00000,
  0b01010,
  0b00000,
  0b00000,
  0b00000,
  0b01110,
  0b10001
};

byte armsDown[8] = {
  0b00100,
  0b01010,
  0b00100,
  0b00100,
  0b01110,
  0b10101,
  0b00100,
  0b01010
};

byte armsUp[8] = {
  0b00100,
  0b01010,
  0b00100,
  0b10101,
  0b01110,
  0b00100,
  0b00100,
  0b01010
};

void setup() {
  // initialize LCD and set up the number of columns and rows: 
  lcd.begin(16, 2);
  
  // create a new character
  lcd.createChar(0, heart);
  // create a new character
  lcd.createChar(1, smiley);
  // create a new character
  lcd.createChar(2, frownie);
  // create a new character
  lcd.createChar(3, armsDown);
  // create a new character
  lcd.createChar(4, armsUp);
}

void loop() {
  int delayTime = 400, i;
  
  // print "I <3 Arduino! :)"
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("I "); 
  lcd.write((byte)0); // when calling lcd.write() '0' must be cast as a byte
  lcd.print(" Arduino! ");
  lcd.write((byte)1);
  delay(delayTime*4);
  
  // print greeting
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Hello, C");
  lcd.write((byte)0);
  lcd.print("MP444!");
  lcd.write(1);
  delay(delayTime*2);
  
  while (true) {
    
    // do little man dance
    for (i = 0; i < 6; i++) {
      lcd.setCursor(4, 1);    // char 5, row 2
      lcd.write(3);           // draw the little man, arms down
      delay(delayTime);
      lcd.setCursor(4, 1);    // draw him arms up
      lcd.write(4);
      delay(delayTime);
    }
    
    // do heart beat
    for (i = 0; i < 6; i++) {
      lcd.setCursor(7,1);      // char 8, row 2
      lcd.write(" ");         // beat off
      delay(delayTime/2);
      lcd.setCursor(7,1);
      lcd.write((byte)0);      // beat on
      delay(delayTime);
    }
      
    // do smiley/frownie
    for (i = 0; i < 6; i++) {
      lcd.setCursor(10,1);     // char 11, row 2
      lcd.write(1);
      delay(delayTime);
      lcd.setCursor(10,1);
      lcd.write(2);
      delay(delayTime);
    }
  }
}
