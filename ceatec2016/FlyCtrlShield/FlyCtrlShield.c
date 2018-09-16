#include "FlyCtrlShield_ide.h"		// Additional Header

unsigned long prev_time;
#define MON	A3

#define LED				26			// pin number of Blue LED
#define ORANGE_LED 		25			// pin number of Blue LED
#define SUBGHZ_CH		36			// channel number (frequency)
#define PANID			0xABCD		// panid
#define HOST_ADDRESS	0x3FF8		// distination address

short data_packet[4];

void setup(void)
{
	int i;
	SUBGHZ_PARAM param;
	for(i=3;i<=8;i++)
	{
		pinMode(i,INPUT_PULLUP);
	}
	digitalWrite(MON,HIGH);
	pinMode(MON,OUTPUT);
	prev_time = 0;
	
	SubGHz.init();					// initializing Sub-GHz
	SubGHz.setAckReq(false);

	pinMode(LED,OUTPUT);			// setting of LED
	pinMode(ORANGE_LED,OUTPUT);			// setting of LED
	digitalWrite(LED,HIGH);			// setting of LED
	Serial.begin(115200);
}



void loop(void)
{
	unsigned long current_time;
	short rx;
	short ry;
	short rz;
	unsigned char sw;
	int i;
	SUBGHZ_MSG msg;
	
	digitalWrite(MON,LOW);
	data_packet[0]=analogRead(A0) - 512;
	data_packet[1]=analogRead(A1) - 512;
	data_packet[2]=analogRead(A2);
	
	sw=0;
	for(i=3;i<=8;i++)
	{
		sw = (sw << 1) + ((digitalRead(i) == LOW) ? 0 : 1);
	}
	sw = (~sw)&0x3F;
	data_packet[3]=(short)sw;
	
	SubGHz.begin(SUBGHZ_CH, PANID,  SUBGHZ_100KBPS, SUBGHZ_PWR_20MW);			// start Sub-GHz
	
	// preparing data
	digitalWrite(LED,LOW);														// LED ON
	while(1)
	{
		msg = SubGHz.send(PANID, HOST_ADDRESS, (unsigned char*)&data_packet, sizeof(data_packet),NULL);		// send data
		if(msg == SUBGHZ_TX_CCA_FAIL)
		{
			digitalWrite(ORANGE_LED,LOW);
		}
		else
		{
			digitalWrite(ORANGE_LED,HIGH);
			break;
		}
	}
	digitalWrite(LED,HIGH);														// LED off
	Serial.print_long((long)data_packet[0],DEC);
	Serial.print(",");
	Serial.print_long((long)data_packet[1],DEC);
	Serial.print(",");
	Serial.print_long((long)data_packet[2],DEC);
	Serial.print(",");
	Serial.print_long((long)data_packet[3],DEC);
	Serial.println("");
	// close
	SubGHz.close();																// Sub-GHz module sets into power down mode.
	delay(40);

}
