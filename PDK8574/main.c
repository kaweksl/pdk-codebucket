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

#define SET_I2C_TIMEOUT 1
#define SET_I2C_STOP 1
#define SET_I2C_CONFLICT 0

#define OWN_SENDI2CDATA
#define OWN_SENDACK

#define i2cAddr (uint8_t)(0x25<<1)
#include "i2c.c"

#define INTPIN 3
//Reset interupt - set pin as input with pull-up
#define RST_INT() \
	__set0(PAC,INTPIN); \
	__set1(PAPH,INTPIN);
//Set interrupt - pin as output low
#define SET_INT() \
	__set1(PAC,INTPIN); \
	__set0(PA, INTPIN);


uint8_t desiredState = 0xFF; //Used for pin change interrupt

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
		I2CSTART();
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
		CLEANRETURNBLOCK();
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