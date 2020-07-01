/*
Copyright (C) 2020  Slawek Kawa  http://skawa.net

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* **
Work in progress

I2c Controller for PWM FAN (PC FAN with 4 pin's)

** */

#include <stdint.h>
#include <stdio.h>
#include "pdk.h"
#define __reset()      __asm__("reset\n")

#define CLR_BIT(reg,bit)	reg &= (uint8_t)~(1<<bit)

//interrupt request definitions
#define BIT0	(uint8_t) 0x01
#define BIT1	(uint8_t) 0x02
#define BIT2	(uint8_t) 0x04
#define BIT3	(uint8_t) 0x08
#define BIT4	(uint8_t) 0x10
#define BIT5	(uint8_t) 0x20
#define BIT6	(uint8_t) 0x40
#define BIT7	(uint8_t) 0x80

#define DEBUGPIN 6
#define DEBUGPULSE() 	__set1(PA,DEBUGPIN);\
						__set0(PA,DEBUGPIN);

#define I2C_ADDR (uint8_t)(0x21<<1)

#define SCLPIN 4
#define SDAPIN 0

#define PULLSDA()		__set1(PAC, SDAPIN); \
						__set0(PA, SDAPIN);
						
#define RELEASESDA()	__set0(PAC, SDAPIN);

#define PULLSCL()		__set1(PAC, SCLPIN); \
						__set0(PA, SCLPIN); 
#define RELEASESCL()	__set0(PAC, SCLPIN);

#define SCL ((uint8_t)(PA & (1<<SCLPIN)))
#define SDA ((uint8_t)(PA & (1<<SDAPIN)))

#define FANTACHOPIN 7
#define FANTACHO ((uint8_t)(PA & (1<<FANTACHOPIN)))

typedef enum
{
	R_SPEED=1,            //Read FAN Speed
    S_DUTY,              //Set pwm duty cycle
    R_DUTY,
    RESETDEVICE         //software restart
} eI2CCommands;

typedef enum
{
	TACHO_CALC = BIT0,            //Read FAN Speed
    APPLY_PWM   = BIT1,              //Set pwm duty cycle
    TACHO_GOTFIRST = BIT2     //software restart
} eToDoFlags;

//Variables (and func with variables) that gona be accessd via bit operations (set1 set0 t1sn t0sn)
// have to be in first part of memory (0x00-0x0F - pdk13; 0x3F - pdk14 ; 0x7F - pdk15)
volatile uint8_t I2CData;
volatile uint8_t I2CError; //I2CError bit flags: #0 - waiting timeout
volatile uint8_t ToDoFlags = 0;
void SendByte(uint8_t byte) {
	uint8_t b = 0;
	while(1) {
		if( !(byte & 0x80) ) PULLSDA();	//Pull SDA low if MSB is 0s
		b++;
		byte <<=1;
		while(!SCL);
		while(SCL);	
		if( (byte & 0x80) ) RELEASESDA(); //If next bit is high release SDA
		if (b == 8 ) break;
	}
	RELEASESDA();
}

//Variables that don't need bit access
volatile uint16_t t16count = 0;
volatile uint8_t bytesToSend = 0;

volatile uint8_t intreg[3];	//Internall register

void PassLowSCL() {
    uint8_t count = 0;
    while(!SCL) {
        count++;
        if(count & 0x80) {
            __set1(I2CError, #0);
        }
        if(I2CError) break;
    }
}
void PassHighSCL() {
    uint8_t count = 0;
    while(SCL) {
        count++;
        if(count & 0x80) {
            __set1(I2CError, #0);  
        }
        if(I2CError) break;
    }
}

void sendACK() {
	PULLSDA();    
	PassLowSCL();	//while(!SCL);
	PassHighSCL();	//while(SCL);
	RELEASESDA();    //Release SDA
}



uint8_t GetByte() {
	uint8_t buffer = 0;
	for(uint8_t b=0; b < 8; b++) {
		PassLowSCL();
		buffer = (buffer << 1) | (SDA);
		PassHighSCL();
	}
	//i2cTimeout:;
	return buffer;
}

// https://bisqwit.iki.fi/story/howto/bitmath/
uint16_t divide(uint32_t dividend, uint32_t divisor) {
    if(divisor == 0) return 0;
    uint32_t scaled_divisor = divisor;  // The right-hand side of division
	uint32_t remain         = dividend; // The left-hand side of division, i.e. what is being divided
	uint32_t result   = 0;
	uint32_t multiple = 1;
        
	while(scaled_divisor < dividend)
    //while(!(scaled_divisor & 0x8000))
	// Alternative: while(!(scaled_divisor & 0x8000)) // For 16-bit, test highest order bit.
	// Alternative: while(not_signed(scaled_divisor)) // Same as above.
	{
	    scaled_divisor = scaled_divisor + scaled_divisor; // Multiply by two.
	    multiple       = multiple       + multiple;       // Multiply by two.
	    // You can also use binary shift-by-left here (i.e. multiple = multiple << 1).
	}
	do {
	    if(remain >= scaled_divisor)
	    {
	        remain = remain - scaled_divisor;
	        result = result + multiple;
	    }
	    scaled_divisor = scaled_divisor >> 1; // Divide by two.
	    multiple       = multiple       >> 1;
	} while(multiple != 0);
    return (uint16_t) result;
}

// uint16_t mulint(uint16_t a, uint16_t b) {
// 	uint16_t result = 0;
// 	while (a != 0) {
// 		if(a & 1) {
// 			result = result + b;
// 		}
// 		a = a >> 1;
// 		b = b+b;
// 	}
// 	return result;
// }

unsigned char _sdcc_external_startup(void)
{
  EASY_PDK_FUSE(FUSE_SECURITY_OFF|FUSE_BOOTUP_SLOW);
  EASY_PDK_INIT_SYSCLOCK_8MHZ();                //use 8MHz sysclock
  EASY_PDK_CALIBRATE_IHRC(8000000,4000);        //tune SYSCLK to 8MHz @ 4.000V
  return 0;                                     //perform normal initialization
}

void ISR(void) __interrupt(0)
{
    
	if( INTRQ & INTRQ_PA0 )           //PA0 interrupt request?
	{
		//INTRQ &= ~INTRQ_PA0;        //Clear PA0 interrupt flag
		
		// ** Start condition detection **
		if(!SCL) goto i2cTimeout; 
        I2CError = 0;
        I2CData = 0;
		PassHighSCL();							//Wait for SCL goest down

		// ** Address sampling
		for(uint8_t b=0; b < 8; b++) {
			PassLowSCL();
			I2CData = (I2CData << 1) | (SDA);
			PassHighSCL();
		}
		//We should have 8 bits
		if((I2CData & 0xFE) == I2C_ADDR) { 		//Address match
			sendACK();
		}
		else {
			goto i2cTimeout;
		}			//Addresss mismatch
        
        // ** Read or Write operations
		if(I2CData & 1) { 				//Master Read operation
                for(uint8_t i = 0; i<bytesToSend;i++) {
                    SendByte(intreg[i]);
                    PassLowSCL();
                    PassHighSCL();
                //SendByte(intreg[0]);
                }
                //SendByte(INTRQ);
			} 
		else {            					//Master Write operation
				I2CData = GetByte();
				sendACK();
				switch (I2CData)
				{
					case R_SPEED:
                        __set1(ToDoFlags, #0);
                        //ToDoFlags = 1;
                        //intreg[0]=GetSFR(I2CData);
						break;
					case S_DUTY:
                        TM2B = GetByte();
                        sendACK();
						break;
                    case RESETDEVICE:
                        sendACK();
                        __reset();
						break;
					default: //TODO: Should send NACK ?
						break;
				}
		}
		i2cTimeout:
		RELEASESDA();
        if(INTEN & INTEN_COMP) { //We interrupted freqency count, have to discard it
            __set0(T16M, #5); //Stop Timer
            __set0(ToDoFlags, #2); // Clear TACHO_GOTFIRST
            __asm__(                                 \
            "clear p+0                    \n"\
            "clear p+1                     \n"\
            "stt16 p                     \n"\
            );
            INTRQ &= ~INTRQ_COMP; 
        }
		INTRQ &= ~INTRQ_PA0; //Clear interrupt flag
	} // ** if ( INTRQ & INTRQ_PA0 )

    if(INTEN & INTEN_COMP) {
        if(ToDoFlags & TACHO_GOTFIRST) {
            if(PB & BIT7) {
            __set0(T16M, #5); //Stop Timer
                //Store timer value
                __asm__(                                 \
                "ldt16 p                     \n"\
                "mov a, p+0                    \n"\
                "mov _t16count+0, a                    \n"\
                "mov a, p+1                    \n"\
                "mov _t16count+1, a                    \n"\
                );
                //Clear timer
                __asm__(                                 \
                "clear p+0                    \n"\
                "clear p+1                     \n"\
                "stt16 p                     \n"\
                );
                __set0(ToDoFlags, #2);
                DEBUGPULSE();
            }
        }
        else {
            if(!(PB & BIT7)) {
                __set1(T16M, #5); //Start Timer 
                __set1(ToDoFlags, #2);
                DEBUGPULSE();
            }
        }
        INTRQ &= ~INTRQ_COMP; 
    } // ** if(INTEN & INTEN_COMP) 
    //INTRQ = 0;
} // ** void ISR(void) __interrupt(0)

void main(void)
{

	//Set PA6,PA7 as output and PA0.PA4 as input
	PAC = (BIT6 ) & ~BIT0 & ~BIT4 & ~BIT7 ;
	//PAC = 0xC0;

	//Enable digital input
	PADIER = PADIE_PA0_WAKEUP_ENABLE | PADIE_PA4_WAKEUP_ENABLE | PADIE_PA7_WAKEUP_ENABLE;

	//Setup interrupts
	INTEGS = INTEGS_PA0_FALLING;//Trigger PA0 interrupt on falling edge
	INTEN = INTEN_PA0 | INTEN_COMP;			//Enable PA0 interrupt
	INTRQ = 0;					//Clear interrupt requests

    //Setup timer2 as pwm
    //TM2CT = 0;          //Reset counter
    TM2B = 127;          //Set duty cycle
    TM2S = TM2S_PRESCALE_NONE | TM2S_SCALE_DIV3;
    TM2C = TM2C_CLK_IHRC | TM2C_MODE_PWM | TM2C_OUT_PA3;

    //Setup Timer16 - it will be our frequency counter
    T16M = T16_CLK_DIV64 ;

    //Setup Comparator as PA7 pin change interrupt 
    GPCS = 1 & ~GPCS_COMP_CASE1 ;
    GPCC = GPCC_COMP_ENABLE | GPCC_COMP_PLUS_VINT_R | GPCC_COMP_MINUS_PA7 | GPCC_COMP_RESULT_POSITIV ;

	__engint();                 //Enable interrupt processing
    
	for(;;) //Infinite loop
	{
        if(ToDoFlags) {
            if(ToDoFlags & BIT0) {
                //Hz
                intreg[0] = divide(62500,t16count);
                
                //RPM
                t16count = divide(1875000,t16count);
                intreg[1] = (uint8_t)(t16count>>8);
                intreg[2] = (uint8_t)(t16count);
                bytesToSend = 3;

                __set0(ToDoFlags, #0);
            }
            if(ToDoFlags & BIT1) {

                __set0(ToDoFlags, #1);
            }
        } //if(ToDoFlags)
	} // for loop
}