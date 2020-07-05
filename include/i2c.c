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

#include <stdint.h>
#include <stdio.h>
#include "pdk.h"

// Default settings
#ifndef i2cAddr
	#define i2cAddr (uint8_t)(0x25<<1)
#endif

#ifndef SCLPIN 
	#define SCLPIN 4
#endif

#ifndef SDAPIN
	#define SDAPIN 0
#endif

#if SET_I2C_TIMEOUT != 0
	#define I2C_F_TIMEOUT
#endif
#if SET_I2C_STOP != 0
	#define I2C_F_STOP
#endif
#if SET_I2C_CONFLICT != 0
	#define I2C_F_CONFLICT
#endif

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

#define PULLSDA() \
	__set1(PAC, SDAPIN); \
	__set0(PA, SDAPIN);
						
#define RELEASESDA() \
	__set0(PAC, SDAPIN);

#define PULLSCL() \
	__set1(PAC, SCLPIN); \
	__set0(PA, SCLPIN); 
#define RELEASESCL() \
	__set0(PAC, SCLPIN);


#define SCL ((uint8_t)(PA & (1<<SCLPIN)))
#define SDA ((uint8_t)(PA & (1<<SDAPIN)))

#define DEBUGPIN 6
#define DEBUGPULSE() \
	__set1(PA,DEBUGPIN);\
	__set0(PA,DEBUGPIN);

#ifdef I2C_F_STOP
	#define CLEANRETURNBLOCK() \
			cleanReturn:\
			RELEASESDA();\
			I2CFlags = 0;\
			I2CData = 0;\
			I2CSDA = 0;
#else
	#define CLEANRETURNBLOCK() \
			cleanReturn:\
			RELEASESDA();\
			I2CFlags = 0;\
			I2CData = 0;
#endif


typedef enum
{
	I2C_ABORT =		BIT0,	//Abort flag, set when error or STOP detected
	I2C_STOP =		BIT1,	//Stop detected
	I2C_TIMEOUT =	BIT2,	//Timeout
	I2C_ADDRCOMM =	BIT3,	//Common address used
	I2C_ADDR_MATCH=	BIT4,	//Main address match
	I2C_CONFLICT=	BIT5,	//Conflict detected while sending
} eI2CFlags;	//Bit mask for I2C state flags


//Variables (and func with variables) that gonna be accessed via bit operations (set1 set0 t1sn t0sn)
// have to be in first part of memory (0x00-0x0F - pdk13; 0x3F - pdk14 ; 0x7F - pdk15)
volatile uint8_t I2CData;		//Main buffer for I2C data (destructive at SendI2CData)
volatile uint8_t I2CFlags = 0;	//Flags used in I2C frame, reseted after each frame
#ifdef I2C_F_STOP
	volatile uint8_t I2CSDA = 0;	//Holds SDA state when SCL is low, used to check for STOP condition
#endif
#ifdef I2C_F_TIMEOUT
	uint16_t TimeoutCount;  //Timout counter
#endif

#ifndef OWN_PASSLOWSCL
void PassLowSCL() {
	#ifdef I2C_F_TIMEOUT
		TimeoutCount = 0;
	#endif
	while(!SCL) {
		#ifdef I2C_F_STOP
			I2CSDA = SDA;
		#endif
		#ifdef I2C_F_TIMEOUT
			TimeoutCount++;
			if(TimeoutCount & 0x400) {
				SET_BITMASK(I2CFlags,I2C_ABORT);
				SET_BITMASK(I2CFlags,I2C_TIMEOUT);
			}
			if(I2CFlags & I2C_ABORT) break;
		#endif
	}
}
#endif

#ifndef OWN_PASSHIGHSCL
void PassHighSCL() {
	#ifdef I2C_F_TIMEOUT
		TimeoutCount = 0;
	#endif
	while(SCL) {
		#ifdef I2C_F_STOP
			if(SDA != I2CSDA) {
				SET_BITMASK(I2CFlags,I2C_ABORT);
				SET_BITMASK(I2CFlags,I2C_STOP);
			}
		#endif
		#ifdef I2C_F_TIMEOUT
			if(TimeoutCount & 0x400) {
				SET_BITMASK(I2CFlags,I2C_ABORT);
				SET_BITMASK(I2CFlags,I2C_TIMEOUT);
			}
		#endif
		if(I2CFlags & I2C_ABORT) break;
		#ifdef I2C_F_TIMEOUT
			TimeoutCount++;
		#endif
	}
}
#endif

#ifndef OWN_SENDACK
void sendACK() {
	PULLSDA();    
	PassLowSCL();
	PassHighSCL();
	RELEASESDA();
}
#endif


// Send byte in buffer over i2c, sends MSB first
#ifndef OWN_SENDI2CDATA
void SendI2CData() {
	uint8_t b = 0;
	while(1) {
		if( !(I2CData & 0x80) ) PULLSDA();	//Pull SDA low if MSB is 0
		b++;
		PassLowSCL();
		#ifdef I2C_F_CONFLICT
			//Check if SDA is low when it should't be, indication of another deivce with same addr but higher priority (lower value) data 
			if( (I2CData & 0x80) ) { 
				if(!SDA) {
					SET_BITMASK(I2CFlags,I2C_ABORT);
					SET_BITMASK(I2CFlags,I2C_CONFLICT);
					break;
				}
			}
		#endif
		I2CData <<=1;
		PassHighSCL();
		if( (I2CData & 0x80) ) RELEASESDA(); //If next bit is high release SDA
		if (b == 8 ) break;
	}
	RELEASESDA();
}
#endif
#ifndef OWN_GETBYTE
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
#endif

#define I2CSTART() \
	if(!SCL) goto cleanReturn; \
	PassHighSCL(); \
	GetByte(); \
	if(I2CFlags & I2C_ABORT) goto cleanReturn; \
	uint8_t addrMask = 0xFE; \
	if ((I2CData & addrMask) == i2cAddr) { \
		SET_BITMASK(I2CFlags, I2C_ADDR_MATCH); \
	} \
	if ( !(I2CFlags & I2C_ADDR_MATCH)) goto cleanReturn; \
	sendACK();


		// // ** Start condition detection **
		// if(!SCL) goto cleanReturn; 
		// PassHighSCL();

		// // ** Address sampling
		// GetByte();
        // if(I2CFlags & I2C_ABORT) goto cleanReturn; //If error occured, exit cleanly

		// uint8_t addrMask = 0xFE; //Fixes compiler bug that creates variable for '0xFE', saves 1 byte RAM
		// if ((I2CData & addrMask) == i2cAddr) {
		// 	SET_BITMASK(I2CFlags, I2C_ADDR_MATCH);
		// }
		// if ( !(I2CFlags & I2C_ADDR_MATCH)) goto cleanReturn;
		// sendACK();

