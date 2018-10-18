#include "Read_Ed_Value_ide.h"		// Additional Header

/* FILE NAME: Read_Subc
 * The MIT License (MIT)
GHz. * 
 * Copyright (c) 2015  Lapis Semiconductor Co.,Ltd.
 * All rights reserved.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
*/

#define SUBGHZ_CH	24
#define CH_MIN      35 // 24
#define CH_MAX      60 // 61
#define SUBGHZ_PANID	0xABCD
uint8_t ch;
SUBGHZ_STATUS rx;

#define BLUE_LED	26

void setup(void)
{
	SUBGHZ_MSG msg;

	Serial.begin(115200);
	
	msg = SubGHz.init();
    /*
	if(msg != SUBGHZ_OK)
	{
		SubGHz.msgOut(msg);
		while(1){ }
	}
	
    */
	pinMode(BLUE_LED,OUTPUT);
	digitalWrite(BLUE_LED,HIGH);

	return;
}

void loop(void)
{
	SUBGHZ_MSG msg;
	uint8_t rssi;

//	Serial.print("RSSI:");

    for(ch=CH_MIN; ch < CH_MAX; ch++) {

    	digitalWrite(BLUE_LED, LOW);

    	msg = SubGHz.begin(ch, SUBGHZ_PANID,  SUBGHZ_100KBPS, SUBGHZ_PWR_20MW);
        /*
    	if(msg != SUBGHZ_OK)
    	{
    		SubGHz.msgOut(msg);
    		continue;
    	}
        */
    	msg = SubGHz.rxEnable(NULL);
        /*
    	if(msg != SUBGHZ_OK)
    	{
    		SubGHz.msgOut(msg);
    		continue;
    	}
        */
    	
    	SubGHz.getEdValue(&rssi);
    	Serial.print_long(ch,DEC);
        Serial.print(":");

//    	if (ch > CH_MIN) Serial.print(",");
    	if (rssi < 16)  Serial.print("0x0");
    	else            Serial.print("0x");
    	Serial.print_long(rssi,HEX);

    	Serial.print(",");

    	digitalWrite(BLUE_LED, HIGH);

//   	SubGHz.close();

//    	sleep(10);
    }

    Serial.println("");

	return;
}

