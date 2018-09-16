#include "sensor_controller_ide.h"		// Additional Header

#include "driver_gpio.h"
#include "driver_extirq.h"

/* FILE NAME: Crane.c
 * The MIT License (MIT)
 * 
 * Copyright:	(c) 2015 Lapis Semiconductor
 * Author: 		Naotaka Saito
 * Division:	New Business Development Project
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


/* #############################################################################
Function:
1.setup:
	1.LED BLUE, ORANGE
	2.Gyro callibration
	3.Get gyro callibration data from Flash

2.loop:
	

3.about motor :
3-0. Data from FlyCtrlShield
	motor[0] = level	0 - 1023
	motor[1] = X axis	-512 ~ 511
	motor[2] = Y axis	-512 ~ 511
	motor[3] = switch
		7:	
		6:	
		5:	switch LOW
		4:	switch HIGH
		3:	callibration y+
		2:	callibration y-
		1:	callibration x+
		0:	callibration x-
3-1. Motor data
	Servo motor		pulse width = 1000us - 2000us
					pulse frequency = 24ms
	DC motor		PWM frequency = 10.23ms
3-2.About callibration
	Gyro data is store in Flash memory
	bank	addr	data
	0		0		0x3110 (actibation data)
	0		1		servo X axis callibration
	0		2		servo Y axis callibration
	
	unit of callibration = 8us
	data = -62~+62 ~  (-496us ~ + 496us)
4. mode
  mode=1:   callibration mode connection and DC motor=0 during 1 sec.
  mode=0:   normal mode
  mode=-1:  connection error. need DC motor=0 for return normal mode
############################################################################# */

#define DEBUG
// #define SENSOR_LOGGER

//#################### Definitions ##################################
// Sub-GHz
#define SUBGHZ_CH	58
#define SUBGHZ_PANID	0xF000
#define SUBGHZ_GATEWAY	0xAC8E
#define SUBGHZ_ORIZURU	1
uint8_t rx_data[256];
uint8_t tx_data[256];

// LED 
#define BLUE_LED	26
#define ORANGE_LED	25

#define WEARABLE_FLAG 0x40
#define HOVERING_FLAG 0x20
#define HOVERING_HALT 0x10


#define MOTOR_PS	17
#define MOTOR_PWM	2


// ####### SENSOR DATA
#define CALLIBRATION_CYCLE		25
static float degPitch,degRoll,kPitch,kRoll;
static float kxg03_bias[6];
static KALMAN_PARAM kalman_param[2];
static bool  kxg03_irq = false;
static float val[6];
static bool update_sensor_flag=false;
static bool control = false;

// ###### for debug
volatile unsigned long t0,t1,t2,t3;

void kxg03_isr(void)
{
	kxg03_irq = true;
}
void initSensor(void)
{
	int i,j;
	byte rc;
	uint8_t data;
	// check exist sensor
	rc = kxg03.init(KXG03_DEVICE_ADDRESS_4E);

//	Wire.setTimeout(5);

	// set KXG03 to stand-by
	data = 0xEF;
	kxg03.write(KXG03_STDBY,&data,1);
	
	// clear int1
	kxg03.read(KXG03_INT1_L,&data,1);

	// opem intmask of ACC and GYRO
	data = 0x03;
	kxg03.write(KXG03_INT_MASK1,&data,1);

	// set interrupt mode
	data = 0x0E;
	kxg03.write(KXG03_INT_PIN_CTL,&data,1);
	
	// Enable interrupt from ACC, GYRO
	data = 0x03;
	kxg03.write(KXG03_INT_PIN1_SEL,&data,1);

	// set ACC ODR to 100Hz
	data = 0x74;
	kxg03.write(KXG03_ACCEL_ODR_WAKE,&data,1);
	
	// set GYRO ODR to 
	data = 0x04;
	kxg03.write(KXG03_GYRO_ODR_WAKE,&data,1);
	
	// set Interrupt
	drv_pinMode(9,INPUT);
	drv_attachInterrupt(9,6, kxg03_isr, RISING,false, false);

	// Start sensor
	data = 0xec;
	kxg03.write(KXG03_STDBY,&data,1);

	// sensor callibration
	for(i=0;i<6;i++)
	{
		kxg03_bias[i]=0;
	}
	for(i=0 ; i < CALLIBRATION_CYCLE ; i++)
	{		
		wait_event(&kxg03_irq);
		kxg03_irq=false;
		
		rc=kxg03.read(KXG03_INT1_L,&data,1);
		Serial.print("KXG03_INT1_L");
		Serial.println_long(data,HEX);
		
		rc=kxg03.read(KXG03_INT1_SRC1,&data,1);
		Serial.print("KXG03_INT1_SRC1");
		Serial.println_long(data,HEX);
		
		rc = kxg03.get_val(val);
		Serial.print_long((long)data,HEX);
		for(j=0;j<6;j++)
		{
			kxg03_bias[j] += val[j];
			Serial.print("SENSOR\t");
			Serial.print_double(val[j],2);
		}
		Serial.println("");
		Serial.print("end\t");
		rc=kxg03.read(KXG03_INT1_L,&data,1);
		Serial.print_long((long)data,HEX);
		Serial.print("\t");
		rc=kxg03.read(KXG03_INT1_SRC1,&data,1);
		Serial.println_long((long)data,HEX);
		
		rc = kxg03.get_val(val);
	}
	// GYRO offset
	kxg03_bias[0] = kxg03_bias[0]/CALLIBRATION_CYCLE;
	kxg03_bias[1] = kxg03_bias[1]/CALLIBRATION_CYCLE;
	kxg03_bias[2] = kxg03_bias[2]/CALLIBRATION_CYCLE;

	// starting positopn
	kxg03_bias[3] = kxg03_bias[3]/CALLIBRATION_CYCLE;
	kxg03_bias[4] = kxg03_bias[4]/CALLIBRATION_CYCLE;
	kxg03_bias[5] = kxg03_bias[5]/CALLIBRATION_CYCLE;

	// calcurate default angle
	degPitch = atan(kxg03_bias[5] / sqrt(sq(kxg03_bias[3]) + sq(kxg03_bias[4]))) * RAD_TO_DEG;
	degRoll  = atan(kxg03_bias[4]/ kxg03_bias[3]) * RAD_TO_DEG;

	// initialize kalman filter
	kalman.init(&kalman_param[0]);
	kalman.init(&kalman_param[1]);
	kalman.set(&kalman_param[0],degPitch);
	kalman.set(&kalman_param[1],degRoll);
}
void getSensor(void)
{
	byte rc;
	float mag;
	static unsigned long lastst_detect_time = 0;
	rc = kxg03.get_val(val);
	mag = sqrt(sq(val[3])+sq(val[4])+sq(val[5]))-1;
	if(mag>1.5) 
	{
		if((millis()-lastst_detect_time)>1000)
		{
			if(control) control = false;
			else control = true;
			
			lastst_detect_time = millis();
		}
	}
	degPitch = atan(val[4] / sqrt(sq(val[3]) + sq(val[5]))) * RAD_TO_DEG;
	degRoll  = atan(val[5]/ fabs(val[3])) * RAD_TO_DEG;
//	degRoll  = atan(val[5]/  val[3]) * RAD_TO_DEG;
//	degRoll  = atan(val[4]/  val[5]) * RAD_TO_DEG;
/*
	val[0] =(val[0] - kxg03_bias[0]);		//gx
	val[1] =(val[1] - kxg03_bias[1]);		//gy
	val[2] =(val[2] - kxg03_bias[2]);		//gz
*/
	kPitch=kalman.cal(&kalman_param[0],degPitch,val[2]);
	kRoll=kalman.cal(&kalman_param[1],degRoll,val[1]*-1);			// ブレスレットタイプはジャイロの軸が反転している
}

/*!
 * @brief about setup
 * Initializing function
 * 
 * initializing Serial, ED, Motor, KXG03(Gyro,Acc) sensor,
 * load motor offset
 * @param  none
 * @return none
 */
void setup(void)
{
	SUBGHZ_MSG msg;
//	long myAddress;
	SUBGHZ_PARAM param;
	byte rc;
	short val[4];
	
#ifdef DEBUG
	Serial.begin(115200);
#endif
	
	// ########### initializing LEDs ############
	pinMode(BLUE_LED,OUTPUT);
	pinMode(ORANGE_LED,OUTPUT);
	
	// ########### initializing motor ############
	Wire.begin();
	rc = initSensor();
	
	// ########### initializing Sub-GHz ############
	// Initializing Sub-GHz	
	msg = SubGHz.init();

	msg = SubGHz.begin(SUBGHZ_CH, SUBGHZ_PANID,  SUBGHZ_100KBPS, SUBGHZ_PWR_20MW);
	
	// ########### Change Sub-GHz mode ############
	// Change Sub-GHz mode to non-ack(AddrMode=4)
	SubGHz.setAckReq(false);
//	SubGHz.getSendMode(&param);
//	param.addrType = 4;
//	SubGHz.setSendMode(&param);
	
	val[0]=0;
	val[1]=0;
	val[2]=0;
	val[3]=0;

	SubGHz.send(SUBGHZ_PANID,SUBGHZ_ORIZURU,(unsigned char*)val,sizeof(val),NULL);
//	drv_pinMode(11,OUTPUT);
	drv_digitalWrite(11,LOW);

	// Motor LSI Setting
	digitalWrite(MOTOR_PWM, LOW);
	digitalWrite(MOTOR_PS, HIGH);
	
	pinMode(MOTOR_PWM,OUTPUT);
	pinMode(MOTOR_PS,OUTPUT);
	hhb.init(0,1023);
	hhb.attach(0,16,8);
	hhb.write(0,0);
	hhb.start(0);
	hhb.update();

	return;
}

/*!
 * @brief about loop
 * main function of crane.
 * 
 * - ledaing data from SubGHz
 * - state machine
 *    1. func_waiting_zero
 *    when rf is disconnected or reset module, controller of slider must be 0 at first.
 *    2..func_normal
 *    normal function
 *      2-1. 
 * ,
 * load motor offset
 * @param  none
 * @return none
 */
void loop(void)
{
	static long blue_led_pwm = 0;
	static char blue_led_state = 0;
	static long blue_led_time = 0;
	static long last_tx_time = 0;
	long current_time;
	short rc[4];
	long data;
	float ftmp;
	SUBGHZ_MSG msg;
	// ########### getSensor ############
	if(kxg03_irq)
	{
		kxg03_irq=false;
		getSensor();
		update_sensor_flag=true;
		
	// ########### send SensorData ############
		if(control){
			drv_digitalWrite(11,HIGH);

			rc[0] = kRoll*10;
		
			if(rc[0]<-512) rc[0]=-511;
			else if(rc[0]>=511) rc[0]=511;
		
//			rc[1] = map(-20,0,-90,-512,511);
			rc[1] = -511;

			ftmp = kPitch;
			if(ftmp < -45) ftmp -45; 
			rc[2] = map(kPitch,45,-90,400,900);
			if(rc[2]<0) rc[2]=0;
			if(rc[2]>1023) rc[2]=1023;
			
			rc[3] = WEARABLE_FLAG | HOVERING_FLAG;
						
			if(kRoll>0) digitalWrite(BLUE_LED,LOW);
			else if(kRoll<0) digitalWrite(ORANGE_LED,LOW);
			
			while(1)
			{
				SubGHz.send(SUBGHZ_PANID,SUBGHZ_ORIZURU,(unsigned char*)rc,sizeof(rc),NULL);
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

			Print.init(tx_data,sizeof(tx_data));

			Print.p("STX,");


			Print.f(kPitch,0);			
			Print.p(",");
			Print.f(kRoll,0);
			Print.p(",");

			Print.f(degPitch,0);			
			Print.p(",");
			Print.f(degRoll,0);
			Print.p(",");
/*
			Print.f(val[0],2);
			Print.p(",");
			Print.f(val[1],2);
			Print.p(",");
			Print.f(val[2],2);
			Print.p(",");
			Print.f(val[3],2);
			Print.p(",");
			Print.f(val[4],2);
			Print.p(",");
			Print.f(val[5],2);
			Print.p(",");
*/
/*			Print.l(rc[0],DEC);
			Print.p(",");
			Print.l(rc[1],DEC);
			Print.p(",");
			Print.l(rc[2],DEC);
			Print.p(",");
			Print.l(rc[3],HEX);
*/
			current_time = millis();
			
//			Print.p(",");

			Print.l(current_time-last_tx_time,DEC);
			
			last_tx_time = current_time;
			

			
			Print.p(",ETX");
			Print.ln();

			Serial.write(tx_data,Print.len());
	
	//		SubGHz.send(0xabcd,SUBGHZ_GATEWAY,tx_data,Print.len(),NULL);
			
			digitalWrite(BLUE_LED,HIGH);
			digitalWrite(ORANGE_LED,HIGH);

			switch(blue_led_state)
			{
			case 0:
				blue_led_pwm += 10;
				if(blue_led_pwm>=200)
				{
					blue_led_pwm = 200;
					blue_led_state = 2;
					blue_led_time = millis();
				}
				break;
			case 1:
				if(millis() - blue_led_time>500)
				{
					blue_led_state = 2;
				}
				break;
			case 2:
				blue_led_pwm -=10;
				if(blue_led_pwm <= 0)
				{
					blue_led_pwm = 0;
					blue_led_state = 3;
					blue_led_time = millis();
				}
				break;
			case 3:
				if(millis() - blue_led_time>1000)
				{
					blue_led_state = 0;
				}
				break;
			default:
				blue_led_pwm = 0;
				blue_led_state = 0;
				break;
			}
			hhb.write(0,blue_led_pwm);
			hhb.update();
			
		}
		else {
			drv_digitalWrite(11,LOW);			
			blue_led_pwm = 0;
			blue_led_state = 0;
			hhb.write(0,blue_led_pwm);
			hhb.update();
		}
	}

/*
#ifdef DEBUG
	{
		static uint32_t last_print_time = 0;
		if(( fly_param.last_recv_time - last_print_time) > 1000)
		{
			Serial.print(print_mode[fly_param.func_mode]);
			Serial.print("\t");
			Serial.print_long(en-st,DEC);
			Serial.print("\t");
			Serial.print_long((long)fly_param.motor[0],DEC);
			Serial.print("\t");
			Serial.print_long((long)fly_param.motor[1],DEC);
			Serial.print("\t");
			Serial.print_long((long)fly_param.motor[2],DEC);
			Serial.print("\t");
			Serial.println_long((long)fly_param.motor[3],DEC);
			last_print_time = fly_param.last_recv_time;
		}
	}
#endif
*/
}

