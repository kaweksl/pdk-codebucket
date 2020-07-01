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
Basic implementaion of I2C slave device


** */

#include <stdint.h>
#include <stdio.h>
#include "pdk.h"

#define I2C_ADDR (uint8_t)(0x25<<1)

#define SCLPIN 4
#define SDAPIN 0
#define DEBUGPIN 6


#define BIT0	(uint8_t) 0x01
#define BIT1	(uint8_t) 0x02
#define BIT2	(uint8_t) 0x04
#define BIT3	(uint8_t) 0x08
#define BIT4	(uint8_t) 0x10
#define BIT5	(uint8_t) 0x20
#define BIT6	(uint8_t) 0x40
#define BIT7	(uint8_t) 0x80

#define __reset()      __asm__("reset\n")
#define SET_BIT(reg,bit)			reg |= (uint8_t)(1<<bit)
#define SET_BITMASK(reg,bitmask)	reg |= bitmask
#define TGL_BIT(reg,bit)			reg ^= (uint8_t)(1<<bit)
#define CLR_BIT(reg,bit)			reg &= (uint8_t)~(1<<bit)
#define CLR_BITMASK(reg,bitmask)	reg &= ~bitmask
#define GET_BIT(reg,bit)    		reg & (uint8_t)(1<<bit)

#define PULLSDA()		__set1(PAC, SDAPIN); \
						__set0(PA, SDAPIN);
						
#define RELEASESDA()	__set0(PAC, SDAPIN);

#define PULLSCL()		__set1(PAC, SCLPIN); \
						__set0(PA, SCLPIN); 
#define RELEASESCL()	__set0(PAC, SCLPIN);

#define SCL ((uint8_t)(PA & (1<<SCLPIN)))
#define SDA ((uint8_t)(PA & (1<<SDAPIN)))

#define DEBUGPULSE() 	__set1(PA,DEBUGPIN);\
						__set0(PA,DEBUGPIN);

//I2C commands list
typedef enum
{
	R_VALUE=1,          //Read Value (sends value into sendbuffer)
    W_VALUE,            //Set value
    RESETDEVICE         //software restart
} eI2CCommands;

typedef enum
{
	I2C_ABORT =		BIT0,	//Abort flag, set when error or STOP detected
	I2C_STOP =		BIT1,	//Stop detected
    I2C_TIMEOUT =	BIT2,	//Timeout
	I2C_ADDRCOMM =	BIT3,	//Common address used
	I2C_ADDR_MATCH=	BIT4,
} eI2CFlags;	//Bit mask for I2C state flags

//Variables (and func with variables) that gona be accessd via bit operations (set1 set0 t1sn t0sn)
// have to be in first part of memory (0x00-0x0F - pdk13; 0x3F - pdk14 ; 0x7F - pdk15)
volatile uint8_t I2CData;		//Main buffer for I2C data
volatile uint8_t I2CFlags = 0;	//Flags used in I2C frame, reseted after each frame



//Variables that don't need bit access
volatile uint8_t SendBuffer;
volatile uint8_t intreg[2];	//Internall register

void PassLowSCL() {
    uint8_t count = 0;
    while(!SCL) {
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
        
        if(count & 0x80) {
            SET_BITMASK(I2CFlags,I2C_ABORT);
			SET_BITMASK(I2CFlags,I2C_TIMEOUT);
        }
        if(I2CFlags & I2C_ABORT) break;
		count++;
    }
}

void SendI2CData()  {
	uint8_t b = 0;
	while(1) {
		if( !(I2CData & 0x80) ) PULLSDA();	//Pull SDA low if MSB is 0
		b++;
		PassLowSCL();
		I2CData <<=1;
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
}

// GetByte samples incomig bit, that routine must be quite fast
// Higly depends on peephole optimization, won't work without it
void GetByte() {
	I2CData = 1;	 //That 1 in carry flag gonna mean that we have 8 bits
	while(1) {
		PassLowSCL();
		I2CData = (I2CData << 1) | (SDA);
		if (FLAG & 0x02) break; //Check if carry flag is 1 aka buffer inital 1 has beed shifted out, requires that bit shift above is done by 'SLC' asm ins.
		PassHighSCL();
		if(I2CFlags & I2C_ABORT) return;
	}
	PassHighSCL();
}


unsigned char _sdcc_external_startup(void)
{
  EASY_PDK_FUSE(FUSE_SECURITY_OFF|FUSE_BOOTUP_SLOW);
  EASY_PDK_INIT_SYSCLOCK_8MHZ();
  EASY_PDK_CALIBRATE_IHRC(8000000,4000);
  return 0;
}

void ISR(void) __interrupt(0)
{
    
	if( INTRQ & INTRQ_PA0 )                       //PA0 interrupt request?
	{
		// ** Start condition detection **
		if(!SCL) goto cleanReturn; 
 		PassHighSCL();							//Wait for SCL goest down

		// ** Address sampling
		GetByte(); //Gets byte into I2CData buffer
		if(I2CFlags & I2C_ABORT) goto cleanReturn; //If error occured, exit cleanly

		//We should have 8 bits
		uint8_t addrMask = 0xFE; //Fixes compiler bug that creates variable for '0xFE', saves 1 byte RAM
		if ((I2CData & addrMask) == I2C_ADDR) { //Address matching, 
			SET_BITMASK(I2CFlags, I2C_ADDR_MATCH);
		}
		if ( !(I2CFlags & I2C_ADDR_MATCH)) goto cleanReturn; //Frame not for us
		sendACK();

        // ** Read or Write operations
		if(I2CData & 1) { 				//Master Read operation
				I2CData = SendBuffer;
                SendI2CData();
		} 
		else {            				//Master Write operation
				GetByte();
                if(I2CFlags & I2C_ABORT) goto cleanReturn;
				switch (I2CData)
				{
					case R_VALUE:
                        sendACK();
                        SendBuffer=intreg[0];
						break;
					case W_VALUE:
                        sendACK();
						GetByte();
						intreg[0] += I2CData;
                        sendACK();
						break;
                    case RESETDEVICE:
                        sendACK();
                        __reset();
						break;
					default:
						break;
				}
		}
		cleanReturn:
		RELEASESDA();
		I2CFlags = 0;
        I2CData = 0;
		INTRQ &= ~INTRQ_PA0; //Clear interrupt flag
	} // ** if ( INTRQ & INTRQ_PA0 )
    //INTRQ = 0;
} // ** void ISR(void) __interrupt(0)

void main(void)
{

	//PA0.PA4 as input
	PAC = 0 | (1<<DEBUGPIN) & ~(1<<SCLPIN) & ~(1<<SDAPIN);

	//Enable digital input only on pins PA0 PA4
	PADIER = PADIE_PA0_WAKEUP_ENABLE | PADIE_PA4_WAKEUP_ENABLE;

	//Setup interrupts
	INTEGS = INTEGS_PA0_FALLING;            //Trigger PA0 interrupt on falling edge (SDA)
	INTEN = INTEN_PA0 | INTEN_COMP;			//Enable PA0 interrupt
	INTRQ = 0;					            //Clear interrupt requests

	__engint();                 //Enable interrupt processing
    
	for(;;) //Infinite loop
	{
        
	} // for loop
}