#include <Wire.h> 
//https://github.com/johnrickman/LiquidCrystal_I2C
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x21,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

uint32_t counter = 0;

void setup()
{
  lcd.init();                      // initialize the lcd 
  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(1,0);
  lcd.print("Hello, world!");
  lcd.setCursor(2,1);
  lcd.print("PDK8574");
  delay(5000);
  lcd.clear();
  lcd.setCursor(1,0);
  lcd.print("Counter!");
  lcd.setCursor(0,1);
  lcd.print("                ");
  delay(1);
}


void loop()
{
  lcd.setCursor(1,1);
  lcd.print(counter);
  counter++;
  delay(10);
}
