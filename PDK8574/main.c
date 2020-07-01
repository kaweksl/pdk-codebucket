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

/*
**	PDK8574 is I2C GPIO expander that simulate PCF8574 IC

	It is not 100% same like PCF8574 as there are differences in hardware
	like PDK has weaker pull-ups, there is not initial 'burst' when pin is changed to high

	This is not 100% tested, no warranty
	
	Tested on PFS154, but my goal is to use it with PMS152-S16 and other Padauk's S16
	Tested by driving 1602 LCD with Arduino and software designed for driving PFC8574, without modification

	Whole PB port is dedicated to IO expansion

	Works on 100kHz I2C freq only
	SDA - PA0
	SCL - PA4
	INT - PA3 - interrupt - active low

	Address selecting through pins (A0, A1, A2) not implemented (yet ?)
	It's not pin compatible with PFS8574

	TODO:
		- Address change by pins
		- Adding another address so we could extend capabilities beyond (like address changing)

	
*/

#include <stdint.h>
#include <stdio.h>
#include "pdk.h"

#define i2cAddr (uint8_t)(0x21<<1)

#define __reset()      __asm__("reset\n")

#define SET_BIT(reg,bit)			reg |= (uint8_t)(1<<bit)
#define SET_BITMASK(reg,bitmask)	reg |= bitmask
#define TGL_BIT(reg,bit)			reg ^= (uint8_t)(1<<bit)
#define CLR_BIT(reg,bit)			reg &= (uint8_t)~(1<<bit)
#define CLR_BITMASK(reg,bitmask)	reg &= ~bitmask
#define GET_BIT(reg,bit)    		reg & (uint8_t)(1<<bit)

#define BIT0	(uint8_t) 0x01
#define BIT1	(uint8_t) 0x02
#define BIT2	(uint8_t) 0x04
#define BIT3	(uint8_t) 0x08
#define BIT4	(uint8_t) 0x10
#define BIT5	(uint8_t) 0x20
#define BIT6	(uint8_t) 0x40
#define BIT7	(uint8_t) 0x80

#define PULLSDA()		__set1(PAC, SDAPIN); \
						__set0(PA, SDAPIN);
						
#define RELEASESDA()	__set0(PAC, SDAPIN);

#define PULLSCL()		__set1(PAC, SCLPIN); \
						__set0(PA, SCLPIN); 
#define RELEASESCL()	__set0(PAC, SCLPIN);

#define SCLPIN 4
#define SDAPIN 0
#define INTPIN 3

#define SCL ((uint8_t)(PA & (1<<SCLPIN)))
#define SDA ((uint8_t)(PA & (1<<SDAPIN)))

//We have totem-pole output with optional pull-ups 
//we have to simulate open-drain output
//Reset interupt - set pin as input with pull-up
#define RST_INT()	__set0(PAC,INTPIN); \
					__set1(PAPH,INTPIN);
//Set interrupt - pin as output low
#define SET_INT()	__set1(PAC,INTPIN); \
					__set0(PA, INTPIN);


#define DEBUGPIN 6
#define DEBUGPULSE() 	__set1(PA,DEBUGPIN);\
						__set0(PA,DEBUGPIN);

//EASY_PDK_SERIAL(serialnr);
typedef enum
{
	I2C_ABORT =		BIT0,	//Abort flag, set when error or STOP detected
	I2C_STOP =		BIT1,	//Stop detected
    I2C_TIMEOUT =	BIT2,	//Timeout
	I2C_ADDRCOMM =	BIT3,	//Common address used
	I2C_ADDR_MATCH=	BIT4,
} eI2CFlags;	//Bit mask for I2C state flags

//Variables (and func with variables) that gonna be accessed via bit operations (set1 set0 t1sn t0sn)
// have to be in first part of memory (0x00-0x0F - pdk13; 0x3F - pdk14 ; 0x7F - pdk15)
volatile uint8_t I2CData;		//Main buffer for I2C data (destructive at SendI2CData)
volatile uint8_t I2CFlags = 0;	//Flags used in I2C frame, reseted after each frame
volatile uint8_t I2CSDA = 0;	//Holds SDA state when SCL is low, used to check for STOP condition, TODO: find fast methond to comapre bits

//Variables that don't need bit access
//volatile uint8_t intreg[2];	//Internall register
uint8_t desiredState = 0xFF; //Used for pin change interrupt

void PassLowSCL() {
    uint8_t count = 0;
    while(!SCL) {
		I2CSDA = SDA;
        count++;
        if(count & 0x80) {
            SET_BITMASK(I2CFlags,I2C_ABORT);
			SET_BITMASK(I2CFlags,I2C_TIMEOUT);
        }
		
        if(I2CFlags & I2C_ABORT) break;
    }
}

void PassHighSCL() {
    uint8_t count = 0;
    while(SCL) {
		if(SDA != I2CSDA) {
			SET_BITMASK(I2CFlags,I2C_ABORT);
			SET_BITMASK(I2CFlags,I2C_STOP);
			DEBUGPULSE();
		}
        if(count & 0x80) {
            SET_BITMASK(I2CFlags,I2C_ABORT);
			SET_BITMASK(I2CFlags,I2C_TIMEOUT);
        }
        if(I2CFlags & I2C_ABORT) break;
		count++;
    }
}

// Send byte in buffer over i2c, sends MSB first
void SendI2CData() {
	uint8_t b = 0;
	while(1) {
		if( !(I2CData & 0x80) ) PULLSDA();	//Pull SDA low if MSB is 0
		b++;
		I2CData <<=1;
		PassLowSCL();
		if(PB != desiredState) SET_INT(); //Check if some pin didn't changed
		PassHighSCL();
		if( (I2CData & 0x80) ) RELEASESDA(); //If next bit is high release SDA
		if (b == 8 ) break;
	}
	RELEASESDA();
}


void sendACK() {
	PULLSDA();    
	PassLowSCL();
	PassHighSCL();
	RELEASESDA();
	RST_INT();
	desiredState = PB;
}



// GetByte samples incomig bit, that routine must be quite fast
// Higly depends on peephole optimization, won't work without it
void GetByte() {
	I2CData = 1; //That 1 in carry flag gonna mean that we have 8 bits
	while(1) {
		PassLowSCL();
		I2CData = (I2CData << 1) | (SDA);
		if (FLAG & 0x02) break; //Check if carry flag is 1 aka buffer inital 1 has beed shifted out, requires that bit shift above is done by 'SLC' asm ins.
		PassHighSCL();
		if(I2CFlags & I2C_ABORT) break;
	}
	PassHighSCL();
	return;
}



unsigned char _sdcc_external_startup(void)
{
  EASY_PDK_FUSE(FUSE_SECURITY_OFF|FUSE_BOOTUP_SLOW);
  EASY_PDK_INIT_SYSCLOCK_8MHZ();                //use 8MHz sysclock
  EASY_PDK_USE_FACTORY_BGTR()
  EASY_PDK_CALIBRATE_IHRC(8000000,4000);        //tune SYSCLK to 8MHz @ 4.000V
  return 0;                                     //perform normal initialization
}


void ISR(void) __interrupt(0)
{
    
	if( INTRQ & INTRQ_PA0 )                       //PA0 interrupt request?
	{
		// ** Start condition detection **
		if(!SCL) goto cleanReturn; 
		PassHighSCL();

		// ** Address sampling
		GetByte(); //Gets byte into I2CData buffer
        if(I2CFlags & I2C_ABORT) goto cleanReturn;

		uint8_t addrMask = 0xFE; //Fixes compiler bug that creates variable for '0xFE', saves 1 byte RAM
		if ((I2CData & addrMask) == i2cAddr) {
			SET_BITMASK(I2CFlags, I2C_ADDR_MATCH);
		}
		if ( !(I2CFlags & I2C_ADDR_MATCH)) goto cleanReturn;
		sendACK();		
        // ** Read or Write operations
		if(I2CData & 1) {	//Master Read operation
				ReadBegin:
				I2CData = PB;
                SendI2CData(); //Uses I2CData as destructive buffer
				PassLowSCL();
				if(!SDA) {	//Master send ACK so want's more, (sequentail read)
					PassHighSCL();
					goto ReadBegin;
				}
			} 
		else {            	//Master Write operation
				while(1) {
					GetByte();
					if(I2CFlags & I2C_ABORT) goto cleanReturn;
					sendACK();
					uint8_t tmp = 0xFF ^ I2CData;
					PBC = tmp;			//in PBC output is 1
					PBPH = I2CData;
				}
		}
		cleanReturn:
		RELEASESDA();

		I2CFlags = 0;
        I2CData = 0;
		I2CSDA = 0;
		INTRQ &= ~INTRQ_PA0; //Clear interrupt flag
	} // ** if ( INTRQ & INTRQ_PA0 )
    //INTRQ = 0;
} // ** void ISR(void) __interrupt(0)

void main(void)
{

	// IO Setup
	PAC = (BIT6 ) & ~BIT0 & ~BIT4 & ~BIT3; 	//PA0.PA4 as input (I2C), PA3 as INT, PA6 DEBUG
	PADIER = PADIE_PA0_WAKEUP_ENABLE | PADIE_PA4_WAKEUP_ENABLE | PADIE_PA3_ENABLE; //Enable digital input only on pins PA0 PA4
	PBPH = 0xFF;						//Enable port B pull-ups at startup


	//Setup interrupts
	INTEGS = INTEGS_PA0_FALLING;//Trigger PA0 interrupt on falling edge (SDA)
	INTEN = INTEN_PA0;			//Enable PA0 interrupt
	INTRQ = 0;					//Clear interrupt requests
	__engint();                 //Enable interrupt processing
	for(;;) {//Infinite loop
		if(PB != desiredState) {//Check if interrupt should be set
		 	SET_INT();
		}
	} // for loop
}