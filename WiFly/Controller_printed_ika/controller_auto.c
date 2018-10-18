#include "controller_auto_ide.h"		// Additional Header


/*--------------------------------------------------------
 * 自動制御を加えたコントローラ用プログラム
 --------------------------------------------------------*/

#define LED	26
#define ORANGE_LED	25
#define SUBGHZ_CH	52
#define SUBGHZ_PANID	0x2018
#define SUBGHZ_MYADDR	0x0000
#define SUBGHZ_RXADDR	0x0001


short data_packet[4];
uint8_t print_data[66];
int data = 0;
SUBGHZ_STATUS rx;
int step =0;
int step_2 = 0;
void setup() {
	SUBGHZ_MSG msg;
	long myAddress;

	pinMode(2,INPUT);
	pinMode(3,INPUT);
	pinMode(5,INPUT);
	pinMode(6,INPUT);


	msg = SubGHz.init();
	if(msg != SUBGHZ_OK)
	{
		SubGHz.msgOut(msg);
		while(1){ }
	}
	//initializing Sub_GHz
	pinMode(LED, OUTPUT);
	pinMode(ORANGE_LED, OUTPUT);
	digitalWrite(LED, HIGH);

	myAddress = SubGHz.getMyAddress();
	SubGHz.setMyAddress(SUBGHZ_MYADDR);
	SubGHz.setBroadcastEnb(false);
	SubGHz.setAckReq(false);
	Serial.print("myAddress1 = ");
	Serial.println_long(myAddress,HEX);
	msg = SubGHz.begin(SUBGHZ_CH, SUBGHZ_PANID,  SUBGHZ_100KBPS, SUBGHZ_PWR_20MW);
	if(msg != SUBGHZ_OK)
	{
		SubGHz.msgOut(msg);
		while(1){ }
	}
	msg = SubGHz.rxEnable(NULL);
	if(msg != SUBGHZ_OK)
	{
		SubGHz.msgOut(msg);
		while(1){ }
	}

	Serial.begin(115200);
}

short yaw_return;

short get_yaw_stick(){//ジョイスティックの横の値を返す関数　左-->0 真ん中-->128 右-->256

	if(255 - analogRead(A0)/4 < 148 && 255 - analogRead(A0)/4 > 108){//あるていど遊びをもたせてある。真ん中にあるときでも128にならないため。
		return 128;
	}
	else{
		if (255 - analogRead(A0)/4 > 168){ return 168;}
		else {	return 255 - analogRead(A0)/4; }
	}
}

    
short get_pitch_stick(current_value){//ジョイスティックの横の値を返す関数　下-->0 真ん中-->128 上-->256
	return 255 - analogRead(A1)/4;
}


short get_flapping_ratio(){//slide volumeの値を返す関数　下-->0 真ん中-->128 上-->256
	return analogRead(A2)/4;//255 - analogRead(A2)/4;--> 自作コントローラー用
	
}

short get_mode(){
    short mode_return;
    if(digitalRead(3) == HIGH){
        mode_return = 1;
    } else {
        mode_return = 0;
    }
    if(digitalRead(2) == HIGH){
        mode_return = 0;
    }
    return 0;
}

void auto_transition_hover_to_horizontal(short *packet){
	short initial_servo_angle = packet[1];
	if(step <10){
		packet[0] = get_yaw_stick();
		packet[1] = initial_servo_angle + step * step * (230 - initial_servo_angle)/100;
		packet[2] = 250 - 15*step;
		packet[3] = 0;
	}
	else{
		packet[0] = get_yaw_stick();
		packet[1] = 215 + add_servo_angle();
		packet[2] = 115  + (1024 - analogRead(A3)) / 12 +  (128 - get_pitch_stick(0))/4;
		packet[3] = 0;
	}
}

short add_servo_angle(){
	if (digitalRead(6) == HIGH){
		return 39;
	}
	else{
		return 0;
	}
	}


void auto_transition_horizontal_to_hover(short *packet){
	short initial_servo_angle = packet[1];
	short initial_flapping_power = packet[2];
	if(step_2 <10){
		packet[0] = get_yaw_stick();
		packet[1] = initial_servo_angle - step_2 * initial_servo_angle/10 + 20;
		packet[2] = initial_flapping_power + step_2 *  (200 - initial_flapping_power)/10 + (128 - get_pitch_stick())/4;
		packet[3] = 0;
		delay(150);
	}
	else{
		packet[0] = get_yaw_stick();
		if (20 + step_2*5 > 80){ packet[1] = 80;}
		else {packet[1] = 20 + step_2 * 5;}
		if (step_2 > 40){packet[2] = 170 + (128 - get_pitch_stick())/4;}
		else{ packet[2] = 220 - step_2;}
		packet[3] = 0;
	}
}


void reform_packet(short *packet){
	int i;
    for(i=0;i<4;i++){
        if(packet[i] > 255){
            packet[i] = 255;
        } else if(packet[i] < 0){
            packet[i] = 0;
        }
    }
}

void communicate_via_serial(){
    short rx_len;
    uint8_t rx_data[256];
    SUBGHZ_MAC_PARAM mac;

    rx_len = SubGHz.readData(rx_data, sizeof(rx_data));
    if(rx_len>50)
    {
        digitalWrite(LED, LOW);
        SubGHz.getStatus(NULL,&rx);										// get status of rx
        SubGHz.decMac(&mac,rx_data,rx_len);		// 受信したrawデータを各種パラメータに分解する機能

        Serial.write(mac.payload, mac.payload_len);

        digitalWrite(LED, HIGH);
        //Serial.print("next");
        //Serial.print_long((long)mac.payload_len, DEC);
    }

}


void recieve_control_data(){
	Serial.read();
}

void loop() {
	short index = 0;
	short yaw = 0;
	short pitch = 0;
	uint32_t now;
	uint32_t last_rx=0;


	int i=0;
	SUBGHZ_MSG msg;
	
	communicate_via_serial();

	now = millis();
	if(now - last_rx > 50) {
		if (digitalRead(5) == HIGH){
			auto_transition_hover_to_horizontal(data_packet);
			reform_packet(data_packet);
			step ++;
		}
		else{
			data_packet[0] = get_yaw_stick();	//joystick 横
			data_packet[1] = 30 + (128 - get_pitch_stick())/5;//joystick 
			data_packet[2] = get_flapping_ratio();	//slidevolume
		    data_packet[3] = get_mode(); //get value from mode switch
		    step = 0;
		    step_2 =0;
	        reform_packet(data_packet); //data_packet[i] の中身が255 以上--> 255 ,0以下--> 0
		}
	
		msg = SubGHz.send(SUBGHZ_PANID, SUBGHZ_RXADDR, (unsigned char*)&data_packet, sizeof(data_packet), NULL);		// send data
	
		//Serial.print_long((long)analogRead(A0), DEC); //yaw
		
		Serial.print("Yaw,");
		Serial.print_long((long)data_packet[0], DEC);
		//Serial.print_long((long)analogRead(A0), DEC);
		Serial.print(",");
		//Serial.print_long((long)analogRead(A1), DEC); //pitch
		Serial.print("servo,");
		Serial.print_long((long)data_packet[1], DEC);
		Serial.print(",");
		//Serial.print_long((long)analogRead(A2), DEC);	//DC
		Serial.print("power,");
		Serial.print_long((long)data_packet[2], DEC);
		Serial.println("");
		Serial.print("analog3,");
		Serial.print_long((long)analogRead(A3), DEC);
		Serial.println("");
		last_rx = now;
	}

	return;
}
