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
I2C Slave device


Implemented Timeout, STOP detection, conflict while sending detection

All i2c communication is done in interrupt, with is fired on falling edge of PA0 (SDA) pin

Tested on PFS154
** */
#include <stdint.h>
#include <stdio.h>
#include "pdk.h"

#define SET_I2C_TIMEOUT 1
#define SET_I2C_STOP 1
#define SET_I2C_CONFLICT 0

#define i2cAddr (uint8_t)(0x25<<1)
#include "i2c.c"

typedef enum
{
	R_VALUE=1,            //Read Value
    W_VALUE,              //Set value
    RESETDEVICE,         //software restart
} eI2CCommands;

volatile uint8_t LastCommand = 0;
volatile uint8_t SendBuffer = 232;
volatile uint8_t intreg[2];	//Internall register to hold some stuff

unsigned char _sdcc_external_startup(void)
{
  EASY_PDK_FUSE(FUSE_SECURITY_OFF|FUSE_BOOTUP_SLOW);
  EASY_PDK_INIT_SYSCLOCK_8MHZ();                //use 8MHz sysclock
  EASY_PDK_CALIBRATE_IHRC(8000000,4000);        //tune SYSCLK to 8MHz @ 4.000V
  return 0;                                     //perform normal initialization
}

void ISR(void) __interrupt(0)
{
    
	if( INTRQ & INTRQ_PA0 )                       //PA0 interrupt request?
	{
		I2CSTART();
        // ** Read or Write operations
		if(I2CData & 1) { 				//Master Read operation
				//if(I2CFlags & I2C_ADDRCOMM) {
				switch (LastCommand)
				{
					default:
						I2CData = SendBuffer;
						SendI2CData();
						break;
				}

		} 
		else {            				//Master Write operation
				GetByte();
                if(I2CFlags & I2C_ABORT) goto cleanReturn;
				switch (I2CData)
				{
					case R_VALUE:
                        sendACK();
						LastCommand = R_VALUE;
                        SendBuffer=intreg[0];
						break;
					case W_VALUE:
                        sendACK();
						GetByte();
						if(I2CFlags & I2C_ABORT) goto cleanReturn;
						sendACK();
                        intreg[0] += I2CData;
						LastCommand = R_VALUE;
						break;
                    case RESETDEVICE:
                        sendACK();
                        __reset();
					default:
						break;
				}
		}
		CLEANRETURNBLOCK();
		INTRQ &= ~INTRQ_PA0; //Clear interrupt flag
	} // ** if ( INTRQ & INTRQ_PA0 )

} // ** void ISR(void) __interrupt(0)

void main(void)
{

	//Initial pins setup
	PAC = (BIT6 ) & ~(1<<SCLPIN) & ~(1<<SDAPIN);

	//Enable digital input only on pins PA0 PA4
	PADIER = PADIE_PA0_WAKEUP_ENABLE | PADIE_PA4_WAKEUP_ENABLE;

	//Setup interrupts
	INTEGS = INTEGS_PA0_FALLING;            //Trigger PA0 interrupt on falling edge (SDA)
	INTEN = INTEN_PA0 | INTEN_COMP;			//Enable PA0 interrupt
	INTRQ = 0;					            //Clear interrupt requests

	__engint();                 //Enable interrupt processing
    
	for(;;) //Infinite loop
	{
		__nop();        
	} // for loop
}