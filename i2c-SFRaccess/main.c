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
With this software you can access SFR registers by I2C
Also you can set or read Timer 16 counter (2 byte)
For driving example check arduino-tests


Tested on PFS154 and PMS150C-u08
** */
#include <stdint.h>
#include <stdio.h>
#include "pdk-includes/device.h"
#include "easy-pdk-includes/calibrate.h"

#define SET_I2C_TIMEOUT 1
#define SET_I2C_STOP 1
#define SET_I2C_CONFLICT 0

#define i2cAddr (uint8_t)(0x25<<1)
#include "i2c.c"

typedef enum
{
	R_VALUE=1,			//Read Value
	W_VALUE,			//Set value
	R_T16,				//Read Timer16 counter
	W_T16,				//Write to Timer16counter
	C_T16,				//Clear t16 counter
	RESETDEVICE,		//software restart
} eI2CCommands;

volatile uint8_t intreg[2];	//Internall register to hold some stuff


#if defined(PFS154)
uint8_t GetSFR(uint8_t addr) {
    switch (addr)
    {
        case 0x00: return FLAG;
        case 0x02: return SP;
        case 0x03: return CLKMD;
        case 0x04: return INTEN;
        case 0x05: return INTRQ;
        case 0x06: return T16M;
        case 0x09: return TM2B;
        case 0x0a: return EOSCR;
        case 0x0b: return IHRCR;
        case 0x0c: return INTEGS;
        case 0x0d: return PADIER;
        case 0x0e: return PBDIER;
        case 0x10: return PA;
        case 0x11: return PAC;
        case 0x12: return PAPH;
        case 0x14: return PB;
        case 0x15: return PBC;
        case 0x16: return PBPH;
        case 0x17: return TM2S;
        case 0x18: return GPCC;
        case 0x19: return GPCS;
        case 0x1a: return BGTR;
        case 0x1b: return MISCLVR;
        case 0x1c: return TM2C;
        case 0x1d: return TM2CT;
        case 0x20: return PWMG0C;
        case 0x21: return PWMG0S;
        case 0x22: return PWMG0DTH;
        case 0x23: return PWMG0DTL;
        case 0x24: return PWMG0CUBH;
        case 0x25: return PWMG0CUBL;
        case 0x26: return PWMG1C;
        case 0x27: return PWMG1S;
        case 0x28: return PWMG1DTH;
        case 0x29: return PWMG1DTL;
        case 0x2a: return PWMG1CUBH;
        case 0x2b: return PWMG1CUBL;
        case 0x2c: return PWMG2C;
        case 0x2d: return PWMG2S;
        case 0x2e: return PWMG2DTH;
        case 0x2f: return PWMG2DTL;
        case 0x30: return PWMG2CUBH;
        case 0x31: return PWMG2CUBL;
        case 0x32: return TM3C;
        case 0x33: return TM3CT;
        case 0x34: return TM3S;
        case 0x35: return TM3B;
        default: return 0;
    }
}
void SetSFR(uint8_t addr, uint8_t value) {
    switch (addr)
    {
        case 0x00: FLAG = 	value; return;
        case 0x02: SP = 	value; return;
        case 0x03: CLKMD = 	value; return;
        case 0x04: INTEN = 	value; return;
        case 0x05: INTRQ = 	value; return;
        case 0x06: T16M = 	value; return;
        case 0x09: TM2B = 	value; return;
        case 0x0a: EOSCR = 	value; return;
        case 0x0b: IHRCR = 	value; return;
        case 0x0c: INTEGS = value; return;
        case 0x0d: PADIER = value; return;
        case 0x0e: PBDIER = value; return;
        case 0x10: PA = 	value; return;
        case 0x11: PAC = 	value; return;
        case 0x12: PAPH = 	value; return;
        case 0x14: PB = 	value; return;
        case 0x15: PBC = 	value; return;
        case 0x16: PBPH = 	value; return;
        case 0x17: TM2S = 	value; return;
        case 0x18: GPCC = 	value; return;
        case 0x19: GPCS = 	value; return;
        case 0x1a: BGTR = 	value; return;
        case 0x1b: MISCLVR =value; return;
        case 0x1c: TM2C = 	value; return;
        case 0x1d: TM2CT = 	value; return;
        case 0x20: PWMG0C = value; return;
        case 0x21: PWMG0S = value; return;
        case 0x22: PWMG0DTH = 	value; return;
        case 0x23: PWMG0DTL = 	value; return;
        case 0x24: PWMG0CUBH = 	value; return;
        case 0x25: PWMG0CUBL = 	value; return;
        case 0x26: PWMG1C = 	value; return;
        case 0x27: PWMG1S = 	value; return;
        case 0x28: PWMG1DTH = 	value; return;
        case 0x29: PWMG1DTL = 	value; return;
        case 0x2a: PWMG1CUBH = 	value; return;
        case 0x2b: PWMG1CUBL = 	value; return;
        case 0x2c: PWMG2C = 	value; return;
        case 0x2d: PWMG2S = 	value; return;
        case 0x2e: PWMG2DTH = 	value; return;
        case 0x2f: PWMG2DTL = 	value; return;
        case 0x30: PWMG2CUBH = 	value; return;
        case 0x31: PWMG2CUBL = 	value; return;
        case 0x32: TM3C = 	value; return;
        case 0x33: TM3CT = 	value; return;
        case 0x34: TM3S = 	value; return;
        case 0x35: TM3B = 	value; return;
        default: return;
    }
}
#elif defined(PMS150C)
uint8_t GetSFR(uint8_t addr) {
    switch (addr)
    {
        case 0x00: return FLAG;
        case 0x02: return SP;
        case 0x03: return CLKMD;
        case 0x04: return INTEN;
        case 0x05: return INTRQ;
        case 0x06: return T16M;
        case 0x09: return TM2B;
        case 0x0a: return EOSCR;
        case 0x0b: return IHRCR;
        case 0x0c: return INTEGS;
        case 0x0d: return PADIER;
        case 0x10: return PA;
        case 0x11: return PAC;
        case 0x12: return PAPH;
        case 0x17: return TM2S;
        case 0x19: return BGTR;
        case 0x1a: return GPCC;
        case 0x1b: return MISC;
        case 0x1c: return TM2C;
        case 0x1d: return TM2CT;
        case 0x1e: return GPCS;
        case 0x1f: return ILRCR;
        default: return 0;
    }
}
void SetSFR(uint8_t addr, uint8_t value) {
    switch (addr)
    {
        case 0x00: FLAG = 	value; return;
        case 0x02: SP = 	value; return;
        case 0x03: CLKMD = 	value; return;
        case 0x04: INTEN = 	value; return;
        case 0x05: INTRQ = 	value; return;
        case 0x06: T16M = 	value; return;
        case 0x09: TM2B = 	value; return;
        case 0x0a: EOSCR = 	value; return;
        case 0x0b: IHRCR = 	value; return;
        case 0x0c: INTEGS = 	value; return;
        case 0x0d: PADIER = 	value; return;
        case 0x10: PA = 	value; return;
        case 0x11: PAC = 	value; return;
        case 0x12: PAPH = 	value; return;
        case 0x17: TM2S = 	value; return;
        case 0x19: BGTR = 	value; return;
        case 0x1a: GPCC = 	value; return;
        case 0x1b: MISC = 	value; return;
        case 0x1c: TM2C = 	value; return;
        case 0x1d: TM2CT = 	value; return;
        case 0x1e: GPCS = 	value; return;
        case 0x1f: ILRCR = 	value; return;
        default: return;
    }
}
#endif

unsigned char _sdcc_external_startup(void)
{
  PDK_SET_FUSE(FUSE_SECURITY_OFF|FUSE_BOOTUP_SLOW);
  PDK_USE_8MHZ_IHRC_SYSCLOCK();                //use 8MHz sysclock
  PDK_USE_FACTORY_BGTR();
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
				I2CData = intreg[0];
				SendI2CData();
				PassLowSCL();
				if(!SDA) {	//Master send ACK so want's more
					I2CData = intreg[1];
					PassHighSCL();
					SendI2CData();
				}

		} 
		else {            					//Master Write operation
				GetByte();
                if(I2CFlags & I2C_ABORT) goto cleanReturn;
				switch (I2CData)
				{
					case R_VALUE:
						sendACK();
						GetByte();
						if(I2CFlags & I2C_ABORT) goto cleanReturn;
						sendACK();
						intreg[0]=GetSFR(I2CData);
						break;
					case W_VALUE:
						sendACK();
						GetByte();
						if(I2CFlags & I2C_ABORT) goto cleanReturn;
						uint8_t sfraddr = I2CData;
						sendACK();
						GetByte();
						if(I2CFlags & I2C_ABORT) goto cleanReturn;
						sendACK();
						SetSFR(sfraddr, I2CData);
						break;
					case R_T16: //Moves timer16 counter to intreg[0 to 1] (2bytes);
						sendACK();
						__asm__(                                 \
						"ldt16 p                     \n"\
						"mov a, p+0                    \n"\
						"mov _intreg+0, a                    \n"\
						"mov a, p+1                    \n"\
						"mov _intreg+1, a                    \n"\
						);
						break;
					case W_T16: //Moves data into timer16 counter;
						sendACK();
						GetByte();
						intreg[0] = I2CData;
						sendACK();
						GetByte();
						intreg[1] = I2CData;
						sendACK();
						__asm__(				\
						"mov a, _intreg+0	\n"\
						"mov p+0, a			\n"\
						"mov a, _intreg+1	\n"\
						"mov p+1, a			\n"\
						"stt16 p			\n"\
						);
						break;
					case C_T16: //Clear T16 counter
						sendACK();
						//T16C = 0; //it compiles but doesn't work, clears only first byte
						__asm__(                                 \
						"clear p+0                    \n"\
						"clear p+1                     \n"\
						"stt16 p                     \n"\
						);
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
    //INTRQ = 0;
} // ** void ISR(void) __interrupt(0)

void main(void)
{

	//Initial pins setup
	PAC = 0xC0;

	//Enable digital input only on pins PA0 PA4
	PADIER = PIN0 | PIN4 | PIN3;

	//Setup interrupts
	INTEGS = INTEGS_PA0_FALLING;            //Trigger PA0 interrupt on falling edge (SDA)
	INTEN = INTEN_PA0 | INTEN_COMP;			//Enable PA0 interrupt
	INTRQ = 0;					            //Clear interrupt requests

	__engint();                 //Enable interrupt processing
    
	for(;;) //Infinite loop
	{
		__set1( PA, 7 );
        
		__set0( PA, 7 );       
	} // for loop
}