// This sketch will send value to i2c slave, send command to process that value (add it to prev value),
// read 1 byte which is result
// at begining and every 10 values it will send reset command

#include <Wire.h>

typedef enum
{
    R_VALUE=1,            //Read Value
    W_VALUE,              //Set value
    RESETDEVICE         //software restart
} eI2CCommands;

int reading = 0;
uint8_t  x = 0;

#define I2C_ADDR 0x25

void setup()
{
  Wire.begin();
  Serial.begin(9600);


  //Send software reset to device
  Wire.beginTransmission(I2C_ADDR);
    Wire.write(RESETDEVICE); 
  Wire.endTransmission();
  delay(100);
}



void loop()
{
  //Send command to increment intreg[0] (W_VALUE) by x
  Wire.beginTransmission(I2C_ADDR);
    Wire.write(W_VALUE);
    Wire.write(x);
  Wire.endTransmission();
  
  Serial.print("Send value\t");
  Serial.print(x);
  Serial.print("\t ");
  delay(1);

  //Send command to move data from intreg[0] to SendBuffer
  Wire.beginTransmission(I2C_ADDR); 
     Wire.write(R_VALUE); 
  Wire.endTransmission();
  delay(1);

  //Read SendBuffer
  Wire.requestFrom(I2C_ADDR, 1);
  delay(5);
  if (Wire.available()) {
    reading = Wire.read();
    Serial.print("Read value ");
    Serial.println(reading);
  } else {
    Serial.println("No response");
  }

  
  x++;
  if(x % 10 == 0) { //Reset every some time
      Wire.beginTransmission(I2C_ADDR); 
        Wire.write(RESETDEVICE);        
      Wire.endTransmission();
      Serial.println("Reset");
  }
  delay(500);
}
