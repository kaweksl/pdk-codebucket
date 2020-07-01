// send only number without line ending
// Send 1 to print SFRs
// send 2 setup timer 2 to output some pwm
// send 3 stop pwm generation

//Note that some registers are write only and reading from them might result in incorrect readout


#include <Wire.h>


typedef enum
{
  R_VALUE=1,            //Read Value
  W_VALUE,              //Set value
  R_T16,        //Read Timer16 counter
  W_T16,        //Write to Timer16counter
  C_T16,        //Clear t16 counter
  RESETDEVICE,         //software restart
} eI2CCommands;

uint16_t reading = 0;
byte x = 60;

#define I2C_ADDR 0x25


void PrintSFRs() {
    for(uint16_t  i=0; i<=0x40; i++) { 
      Wire.beginTransmission(I2C_ADDR); // transmit to device #4
      Wire.write(R_VALUE);
      Wire.write(i);
      Wire.endTransmission();
      delay(1);
     Wire.requestFrom(I2C_ADDR, 1);
     if (Wire.available()) {
        reading = Wire.read();
        Serial.print("SFR: 0x");
        Serial.print(i, HEX);
        Serial.print(" - 0x");
        Serial.println(reading, HEX);
        delay(10);
     }
   };
}

void setup()
{
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(9600);
  Serial.println("    ");
  PrintSFRs();
}


void loop()
{
 while (Serial.available() > 0) {
      x = Serial.parseInt();
      switch(x) {
        case 1:   //List SFR values
          PrintSFRs();
          break;
        case 2:   //will setup timer2 to output some pwm on PA3
           Wire.beginTransmission(I2C_ADDR);
            Wire.write(W_VALUE);
            Wire.write(0x1d); //tm2ct
            
            Wire.write(0);
           Wire.endTransmission();
           delay(2);
        
          Wire.beginTransmission(I2C_ADDR);
            Wire.write(W_VALUE);
            Wire.write(0x09); //tm2b
            //Wire.write( (uint8_t)600 >> 3);
             Wire.write(0x7f);
           Wire.endTransmission();
           delay(2);
        
           delay(2);
           Wire.beginTransmission(I2C_ADDR);
            Wire.write(W_VALUE);
            Wire.write(0x17); //tm2s
            Wire.write(0x02 );
           Wire.endTransmission();
           delay(2);
           
           Wire.beginTransmission(I2C_ADDR);
            Wire.write(W_VALUE);
            Wire.write(0x1c); //tm2c
            //Wire.write( (uint8_t)600 & 0x07);
            Wire.write(0x2a);
           Wire.endTransmission();
           delay(3000);
           break;
        case 3: //Turn off pwm gen
           Wire.beginTransmission(I2C_ADDR);
            Wire.write(W_VALUE);
            Wire.write(0x1c); //tm2s
            Wire.write(0);
           Wire.endTransmission();
           break;
        case 10: //Sets value into t16 counter (2 bytes)
           Wire.beginTransmission(I2C_ADDR);
            Wire.write(W_T16);
            Wire.write(0xCA);
            Wire.write(0xCA);
           Wire.endTransmission();
          break;
        case 11: //move t16 counter into sendbuffer
           Wire.beginTransmission(I2C_ADDR);
            Wire.write(R_T16);
           Wire.endTransmission();
          break;
        case 12: //Read 2 bytes sendbuffer
             Wire.requestFrom(I2C_ADDR, 2);
             delay(1);
            if (Wire.available()) {
              reading = Wire.read();
              reading <<=8;
              reading += Wire.read();
              Serial.print("T16 reading: ");
              Serial.println(reading, HEX);
        
            }
          break;
         case 13: //clear t16 counter
            Wire.beginTransmission(I2C_ADDR);
            Wire.write(C_T16);
           Wire.endTransmission();
          break;
        case 99: //Reset device
           Wire.beginTransmission(I2C_ADDR);
            Wire.write(RESETDEVICE);
           Wire.endTransmission();
          break;
        default:
          break;
      }
    }
    if (Wire.available()) {

      reading = Wire.read();
      Serial.print("Reading: ");
      Serial.println(reading);

    }
}
