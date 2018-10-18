#include "WiFly_v3_ide.h"		// Additional Header

/* Sub-GHz */
uint8_t tx_data[256];
uint8_t rx_data[256];
SUBGHZ_STATUS rx;

#define SUBGHZ_CH	52	//36
#define SUBGHZ_PANID	0x2018
#define SUBGHZ_MYADDR	0x0001
#define SUBGHZ_RXADDR	0x0000
#define SUBGHZ_RATE		100
#define SUBGHZ_PWR		20
/* Sensor */
bool kxg03_irq = false;

/* Motors */
#define MOTOR_PS	17
#define MOTOR_PWM	2

/* LED */
#define BLUE_LED	26


/* posture parameter */
typedef struct{
	float CurrentPosture;
	float TargetPosture;
} Posture;

Posture pitch;
Posture slope;


/* control parameter */
typedef struct{
	float ControlValue;
	float CurrentValue;
	float TargetValue;
	float ControlMode;
} Control;

Control level;
Control servo;
Control yaw;
Control mode;

/* flight parameter */
typedef struct{
	long current_time;	
	long last_recv_time;
	int length;
	int signal_check;
	bool flight_permission;
} FlyParam;

static FlyParam fly_param;



static void kxg03_isr(void){
	kxg03_irq = true;
}


void Emargency_Stop(){
	Serial.println("Emargency Stop!");
	hhb.write(3,0L);
	hhb.write(2,0L);
	hsv.write(0,0);
	hsv.update();

	fly_param.flight_permission = false;
	digitalWrite(BLUE_LED, LOW);

	return;
}


int Time_Update(void){

	if(fly_param.length > 0){
		fly_param.current_time = millis();
		fly_param.last_recv_time = fly_param.current_time;
		digitalWrite(BLUE_LED, HIGH);
		return 1;
	} else {
		fly_param.current_time = millis();
		if(fly_param.current_time - fly_param.last_recv_time > 2000){
			Emargency_Stop();
		}
		return 0;
	}

}


void Signal_Check(){
	SUBGHZ_MAC_PARAM mac;
	fly_param.length = SubGHz.readData(rx_data, sizeof(rx_data));
	fly_param.signal_check = Time_Update();
//	level.ControlValue = map((long)(rx_data[11]),0L,255L,0L,1023L);
	SubGHz.decMac(&mac,rx_data,fly_param.length);
	level.ControlValue = mac.payload[4];	//11
}


void Update_Motor_Data(void){
	SUBGHZ_MAC_PARAM mac;

	fly_param.length = SubGHz.readData(rx_data, sizeof(rx_data));

/*	level.ControlValue = map((long)(rx_data[11]),0L,255L,0L,1023L);
	servo.ControlValue = map((long)(rx_data[9]),0L,255L,900L,1923L);		//(unsigned short)?
	yaw.ControlValue = map((long)(rx_data[7]),0L,255L,-1023L,1023L);*/

	SubGHz.decMac(&mac,rx_data,fly_param.length);

	level.ControlValue = mac.payload[4];	//11
	servo.ControlValue = mac.payload[2];	//9
	yaw.ControlValue = mac.payload[0];		//7

	mode.ControlMode = mac.payload[6];	//13

	return;
}


void Update_Posture_Data_From_Sensor(){
	static float val[16];
	wait_event(&kxg03_irq);
	kxg03.get_angle(val);

	Print.init(tx_data,sizeof(tx_data));
	
	Print.p("STX,");
    Print.d((double)val[0], 2);		// kalman pitch
    Print.p(",");
    Print.d((double)val[1], 2);		// kalman roll
    Print.p(",");
    Print.d((double)val[2], 2);		// acc deg
    Print.p(",");
    Print.d((double)val[3], 2);		// acc deg
    Print.p(",");
    Print.d((double)val[4], 2);		// gyro x
    Print.p(",");
    Print.d((double)val[5], 2);		// gyro y
    Print.p(",");
    Print.d((double)val[6], 2);		// gyro z
    Print.p(",");
    Print.d((double)val[7], 2);		// acc x
    Print.p(",");
    Print.d((double)val[8], 2);		// acc y
    Print.p(",");
    Print.d((double)val[9], 2);		// acc z
    Print.p(",ETX");
    Print.ln();
    
    Serial.print(tx_data);

    //pitch.CurrentValue = (short)val[0];
    //slope.CurrentValue = (short)val[1];

	SubGHz.send(SUBGHZ_PANID, SUBGHZ_RXADDR, tx_data, Print.len(), NULL);

    return;

}


void Motor_Sets(){

/*	hhb.write(3,(long)level.ControlValue);
	hhb.write(2,(long)yaw.ControlValue);
	hsv.write(0,(int)servo.ControlValue);
	hsv.update();*/

	hhb.write(3, map((long)level.ControlValue,0L,255L,0L,1023L));
	hhb.write(2, map((long)yaw.ControlValue,0L,255L,-1023L,1023L));
	hsv.write(0, map((unsigned short)servo.ControlValue,0L,255L,900L,1923L));
	hsv.update();

	return;
}




/*
void Control_Mode_Sets(){
	if(mode.ControlMode == 0){
		
	} else if(mode.ControlMode == 1){
		Emargency_Stop();
	} else {
		Serial.println("mode error!");
	}
}*/



void Flight_Preparation_Mode(){

	while(1){

		Signal_Check();

		if(fly_param.signal_check > 0){
			if(level.ControlValue < 1){
				Serial.println("ready to flight...");
				fly_param.flight_permission = true;
				return;
			} else{
				Serial.println("level is not 0!");
			}
		} else {
			Serial.println("preparation error!");
		}

//	delay(10);
	
	}
}

void Flight_Mode(){
	while(fly_param.flight_permission == true){
		Update_Motor_Data();
		//Update_Posture_Data_From_Sensor();
		Motor_Sets();
	}
}



void setup() {
  // put your setup code here, to run once:
  	byte rc;
	SUBGHZ_MSG msg;
//	SUBGHZ_PARAM param;
	long myAddress;

	Serial.begin(115200);

	// initializing LED
	pinMode(BLUE_LED, OUTPUT);
	digitalWrite(BLUE_LED, HIGH);

	// Motor LES Setting
	digitalWrite(MOTOR_PWM, LOW);
	digitalWrite(MOTOR_PS, HIGH);

	pinMode(MOTOR_PWM, LOW);
	pinMode(MOTOR_PS, HIGH);

	// Servo Motor API Initializings
	level.ControlValue = 0;
	servo.ControlValue = 0;
	yaw.ControlValue = 0;

	hsv.init(2,24000);	//servo
	hsv.attach(0,6);
	hsv.attach(1,7);
	hsv.write(0,(unsigned short)map(0L,-512L,511L,900L,1900L));
	hsv.write(1,1);
	hsv.start(); 
	
	hhb.init(3,1023);	//DCmotor
	hhb.attach(3,9,3);
	hhb.attach(3,16,8);
	hhb.write(3,0L);
	hhb.start(3);

	hhb.init(2,1023);	//yaw
	hhb.attach(2,4,5);
	hhb.write(2,0L);
	hhb.start(2);


	// Initializing Sub-GHz
	msg = SubGHz.init();
	if(msg != SUBGHZ_OK){
		SubGHz.msgOut(msg);
		while(1){}
	}
	
	// Change SubGHz mode to non-ack(AddrMode = 4)
	myAddress = SubGHz.getMyAddress();
	SubGHz.setMyAddress(SUBGHZ_MYADDR);
	SubGHz.setBroadcastEnb(false);

	msg = SubGHz.begin(SUBGHZ_CH, SUBGHZ_PANID, SUBGHZ_100KBPS, SUBGHZ_PWR_20MW);
	if(msg != SUBGHZ_OK){
		SubGHz.msgOut(msg);
		while(1){}
	}

	// Initializing sensor
	Wire.begin();
	rc = kxg03.sync_init(KXG03_DEVICE_ADDRESS_4E, KXG03_ODR_25HZ,kxg03_isr);
	rc = bm1422.init(0);
	rc = bm1383.init(0);
	
	kxg03.angle_init(KXG03_MODE_PR | KXG03_MODE_KALMAN ,3,2,1,2,-3,-1);
	kxg03.set_acc_out(true);
	kxg03.set_gyro_out(true);
	kxg03.set_kalman_out(true);
	kxg03.set_deg_out(true);


	// Enabling RX
	msg = SubGHz.rxEnable(NULL);
	if(msg != SUBGHZ_OK){
		SubGHz.msgOut(msg);
		while(1){}
	}

	// initializing some parameters
	fly_param.length = 0;
	fly_param.current_time = 0;
	fly_param.last_recv_time = 0;
	fly_param.signal_check = 0;
	fly_param.flight_permission = false;
	
	return;
}

	
void loop() {
	Flight_Preparation_Mode();
	Flight_Mode();
}
