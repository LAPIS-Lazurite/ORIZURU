#include "crane_170921_0830_ide.h"		// Additional Header

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

#include "crane.h"

//#################### Definitions ##################################

#define DC1SV2
// #define DC2SV1

#ifdef DC1SV2
	#define SV_NUM 2
#endif
#ifdef DC2SV1
	#define SV_NUM 1
#endif

// Sub-GHz
#define SUBGHZ_CH	58
#define SUBGHZ_PANID	0xF000
#define SUBGHZ_TXADDR	0x7FAD
uint8_t rx_data[256];
uint8_t tx_data[256];
SUBGHZ_STATUS rx;

// Fly Controller
#define FLY_CTRL_MIN	-512L
#define FLY_CTRL_MAX	511L

// Motors
#define MOTOR_PS	17
#define MOTOR_PWM	2
#define SERVO_MIN_TIME	1000L
#define SERVO_MAX_TIME	2000L

#define SERVO_CALIBRATION_UNIT	8		// 1 step -> 8usec
#define SERVO_CALIBRATION_MAX	160		// maximum calibrated time
#define SERVO_CALIBRATION_MIN	SERVO_CALIBRATION_MAX*-1	// minimum calibrated time

// LED 
#define BLUE_LED	26
#define ORANGE_LED	25

#define WEARABLE_FLAG 0x40
#define HOVERING_FLAG 0x20
#define HOVERING_HALT 0x10

#define WEARABLE_FORCE_TIME	200
#define NOSIGNAL_TIME		1000
#define HOVERING_FORCE_TIME	200

static LAZURITE_FLY_PARAM fly_param;

// ####### SENSOR DATA
#define CALLIBRATION_CYCLE		25
static float degPitch,degRoll,kPitch,kRoll;
static float kxg03_bias[6];
static KALMAN_PARAM kalman_param[2];
static bool  kxg03_irq = false;
static float val[6];
static bool update_sensor_flag=false;

static short hovering_motor[4];

// ###### for debug
volatile unsigned long t0,t1,t2,t3;


#ifdef DEBUG
const char print_mode[][16]={
	"Waitint zero",
	"Normal",
	"callibration",
	"hobering",
	"sensor"
};
#endif

typedef struct {
	uint8_t cycle;
	uint8_t sequence[3];
	uint32_t sequence_time[3];
}LED_PARAM;


/*! LED parameter 
 * @brief sequencer of LED
 * 	@code
 * 	fly_param.led.cycle = led_param[3].cycle;
 * 	fly_param.led.sequence = led_param[3].sequence;
 * 	fly_param.led.sequence_time = led_param[3].sequence_time;
 * 	@endcode
 * 	{3,{LED_BOTH,LED_OFF,LED_BOTH},{100,100,100}}
 * 	cycle = 3:   cycle of sequence
 * 	{LED_BOTH,LED_OFF,LED_BOTH}    order of sequence<br>
 * 	{100,100,100}                  sequence time
 * 	(ex)
 * 	LED_BOTH (100ms) --> LED_OFF(100ms) --> LED_BOTH (100ms) --> 
 */
const LED_PARAM led_param[] =
{
	{1,{LED_BOTH,LED_OFF,LED_OFF},{100,0,0}},				// calibration data = 0
	{1,{BLUE_ON,LED_OFF,LED_OFF},{100,0,0}},				// calibration data +
	{1,{ORANGE_ON,LED_OFF,LED_OFF},{100,0,0}},				// calibration data -
	{3,{LED_BOTH,LED_OFF,LED_BOTH},{100,100,100}}			// enter calibration mode
};

// State of detecting zero
FLY_STATE func_waiting_zero(void)
{
	FLY_STATE mode = FLY_STATE_DETECTING_ZERO;
	
	if((fly_param.length > 0) && (fly_param.motor[2] == 0) && (fly_param.motor[3] != HOVERING_HALT))
	{
		mode = FLY_STATE_CALIB;
		fly_param.led.cycle = led_param[3].cycle;
		fly_param.led.sequence = led_param[3].sequence;
		fly_param.led.sequence_time = led_param[3].sequence_time;
		func_motor_callibration();	}
	else
	{
		static unsigned long updated_time = 0x80000000;
		if((fly_param.current_time - updated_time)>100)	// force write 0 to motor once per 100ms
		{
			updated_time = fly_param.current_time;
			fly_param.motor[0] = 0;
			fly_param.motor[1] = 0;
			fly_param.motor[2] = 0;
			fly_param.motor[3] = 0;
			update_motor_data(true);
			digitalWrite(BLUE_LED,HIGH), digitalWrite(ORANGE_LED,LOW);
		}
	}
	return mode;
}

// State of normal
FLY_STATE func_normal(void)
{
	static bool zero_detect = false;
	static uint32_t zero_detect_time = 0;
	FLY_STATE mode = FLY_STATE_NORMAL;
	
	// #### check receiving data or not
	if(fly_param.length <= 0)
	{
		if((fly_param.current_time-fly_param.last_recv_time)>NOSIGNAL_TIME)
		{
		// #### there are no rx for 500ms, go to state of func_waiting_zero
			func_waiting_zero();
			mode = FLY_STATE_DETECTING_ZERO;
		}
	}
	else
	{
		if((fly_param.motor[3] & WEARABLE_FLAG) || 
			((fly_param.motor[3] & HOVERING_FLAG) &&( (fly_param.motor[3] & HOVERING_HALT) == 0)))
			{
				mode = FLY_STATE_HOVERING;
				return mode;
			}
		else
		{
			// #### update morter data 
			memcpy(hovering_motor,fly_param.motor,sizeof(hovering_motor));
//			fly_param.motor[1]-=100;

			if(fly_param.motor[3] & HOVERING_HALT)
			{
				memset(fly_param.motor,0,sizeof(fly_param.motor));
			}
#ifdef DC2SV1
			if(fly_param.motor[1]>0){
				fly_param.motor[1]=0;
			}
#endif
			update_motor_data(true);
			// #### check value of slider motor = 0,
			if(fly_param.motor[2] == 0)
			{
				if(zero_detect == false)
				{
					zero_detect = true;
					zero_detect_time = fly_param.current_time;
				}
				else
				{
					if((fly_param.current_time - zero_detect_time)>5000)
					{
						//###### data is 0 for more than 500ms, state is go to func_motor_callibration
						mode = FLY_STATE_CALIB;
						zero_detect = false;
						fly_param.led.cycle = led_param[3].cycle;
						fly_param.led.sequence = led_param[3].sequence;
						fly_param.led.sequence_time = led_param[3].sequence_time;
						func_motor_callibration();
					}
				}
			}
			else
			{
				zero_detect = false;
			}
		}
	}
	
	return mode;
}

// State of motor callibration
// ## callibration of servo motor
FLY_STATE func_motor_callibration(void)
{
	FLY_STATE mode = FLY_STATE_CALIB;
	
// ## check receiving data
	if(fly_param.length > 0)
	{
		if(fly_param.motor[3] & HOVERING_FLAG)
		{
				mode = FLY_STATE_HOVERING;			
		}
		// ## slider motor is not 0, callibration data is stored in Flash memory and change to func_normal state
		if(fly_param.motor[2] != 0)	// release calibration mode
		{
			update_calib_data();		// 
			mode = FLY_STATE_NORMAL;
			func_normal();
		}
		// ## slider motor is not 0, go to func_normal state
		{
			button_func(button_check((unsigned char)fly_param.motor[3]));
			update_motor_data(true);
		}
	}
	else
	{
		// ## check disconnection
		if((fly_param.current_time-fly_param.last_recv_time)>500)
		{
			func_waiting_zero();
			mode = FLY_STATE_DETECTING_ZERO;
		}
	}
	
	return mode;
}

// ##### Hovering 



/*!
 * ホバリング状態の制御
 * 200msの間、hoveringのフラグを検出しないと通常モードに移行
 * 500msの間、無線を受信しないと制御を0検出モードに移行
 */
static const float kpPitch = 2;
static const float kpRoll = 10;
static uint32_t last_t = 0;								// 前回演算したときの時間
static float vx,vy,vz,vr;
// ###### PID制御用パラメータ
float error_x;							// 現在の横方向の角度の誤差  (ターゲット - 現在の角度)
float error_y;							// 現在の縦方向の角度の誤差  (ターゲット - 現在の角度)
float dt;												// 前回演算してからの経過時間
double derivative_x;									// 微分項
double derivative_y;									//微分項
static short hovering_motor[4];
static short cal_motor[4];
static short pitchOffset=500;

static float lkPitch, dPitch,avrPitchError;
#define pkPitchParam		0.98
#define avrPitchParam		0.99
FLY_STATE func_hovering(void)
{

	// ###### 状態遷移用パラメータ
	FLY_STATE mode = FLY_STATE_HOVERING;
	static uint32_t last_mode_detect_time=0;				//前回無線データを取得した時間
	float targetPitch,targetRoll;

	// ###### 状態遷移
	if(fly_param.length > 0) {
		if(fly_param.motor[3] & HOVERING_FLAG) 
		{
//			Serial.println("Hovering");
			last_mode_detect_time = fly_param.current_time;				// HOVERING_FLAGを検出した時刻を取得
			memcpy(hovering_motor,fly_param.motor,sizeof(hovering_motor));
		}
		if(((fly_param.motor[3] & HOVERING_HALT) == 1) && ((fly_param.motor[3] & WEARABLE_FLAG)==0))
		{
			memset(fly_param.motor,0,sizeof(fly_param.motor));
			mode = FLY_STATE_NORMAL;
			return;
		}
		//end of hovering
		else if(( fly_param.current_time - last_mode_detect_time) > HOVERING_FORCE_TIME)
		{
			mode = FLY_STATE_NORMAL;
			return mode;
		}
	} else
	{
		if((fly_param.current_time-fly_param.last_recv_time)>NOSIGNAL_TIME)
		{
		// #### there are no rx for 500ms, go to state of func_waiting_zero
			func_waiting_zero();
			mode = FLY_STATE_DETECTING_ZERO;
		}
	}
	
	// ###### PID制御用パラメータ

	// ###### PID制御
	if(update_sensor_flag)
	{
		
		targetRoll = map(hovering_motor[0],-512,511,-45,45);
		error_y = (hovering_motor[0]/10+kRoll);							// 現在の縦方向の角度の誤差  (ターゲット - 現在の角度)
		
		targetPitch = map(hovering_motor[1],-512,511,0,-90);
		error_x = (targetPitch - kPitch);							// 現在の横方向の角度の誤差  (ターゲット - 現在の角度)

		// 機体が動く速度を調べる
		dPitch = kPitch - lkPitch;
		// 機体の平均角度を調べる
		avrPitchError = avrPitchError*avrPitchParam+error_x * (1-avrPitchParam);

		lkPitch = kPitch;
		
		// 時間 [s]
		dt = (fly_param.current_time - last_t)/1000;
		last_t = fly_param.current_time;
	
		// 操作量
		vx = kpPitch * error_x;
		vy = kpRoll * error_y;
//		vx = kpPitch * error_x + kiPitch * integral_x + kdPitch * derivative_x;
//		vy = kpRoll * error_y + kiRoll * integral_y + kdRoll * derivative_y;
		vz = 0.0;
		vr = 0.0;		
	//	hovering_motor[0] = vy;
		cal_motor[0] = vy;
	
		if(cal_motor[0] >511 ) cal_motor[0]=511;
		else if(cal_motor[0] < -512 ) cal_motor[0]=-512;
		
	//	fly_param.motor[1] = (vx>0)?vx*-1:0;
#ifdef DC2SV1															// CEATEC 2016
		if(kPitch > -10)
		cal_motor[1] = hovering_motor[2]-pitchOffset;					// 羽モータの値から数値を引いてpitchモーターのオフセットにする
		if(cal_motor[1]<0) cal_motor[1] = 0;
		vx = vx + cal_motor[1];
		vx*=-1;
		
		if(vx>-10) vx = -10;

		cal_motor[1] =vx;
		if(cal_motor[1] >511 ) cal_motor[1]=511;
		else if(cal_motor[1] < -512 ) cal_motor[1]=-512;
#endif
#ifdef DC1SV2															// CEATEC 2017
		cal_motor[1] = hovering_motor[1];								// 水平尾翼は操作の値をそのまま出力する
#endif
		cal_motor[2] = hovering_motor[2];
		cal_motor[3] = hovering_motor[3];
	
		memcpy(fly_param.motor,cal_motor,sizeof(hovering_motor));
		update_motor_data(false);
	}
	
	return mode;
}


// initializing offset parameter for servo motor
void init_servo_offset(void)
{
	unsigned short servo_offset_valid;
	
	// get motor callibration data
	servo_offset_valid = Flash.read(0,0);
	fly_param.servo_offset[0] = Flash.read(0,1);
	fly_param.servo_offset[1] = Flash.read(0,2);
	
	// Check Servo calibration data
	if((servo_offset_valid != 0x3110) ||								// checking motor offset data is invalid
		(fly_param.servo_offset[0] > SERVO_CALIBRATION_MAX ) || (fly_param.servo_offset[0] < SERVO_CALIBRATION_MIN) ||
		(fly_param.servo_offset[1] > SERVO_CALIBRATION_MAX) || (fly_param.servo_offset[1] < SERVO_CALIBRATION_MIN))
	{
		Flash.erase(0);
		servo_offset_valid = 0x3110;
		fly_param.servo_offset[0] = 0;
		fly_param.servo_offset[1] = 0;
		Flash.write(0,0,servo_offset_valid);
		Flash.write(0,1,fly_param.servo_offset[0]);
		Flash.write(0,2,fly_param.servo_offset[1]);
	}
#ifdef DEBUG
	Serial.print("Servo morter offset");
	Serial.print("\tX=");
	Serial.print_long((long)fly_param.servo_offset[0],DEC);
	Serial.print("\tY=");
	Serial.println_long((long)fly_param.servo_offset[1],DEC);
#endif
}

// writing motor data
void update_motor_data(bool offset)
{
	int i;
	long servo[2];
//	Serial.print("M,");
	for(i=0;i<SV_NUM;i++)
	{
		servo[i] = map((long)fly_param.motor[i],FLY_CTRL_MIN,FLY_CTRL_MAX,SERVO_MIN_TIME,SERVO_MAX_TIME);
		servo[i] += fly_param.servo_offset[i];
		if(servo[i] >= SERVO_MAX_TIME) servo[i] = SERVO_MAX_TIME;
		else if (servo[i] <= SERVO_MIN_TIME) servo[i] = SERVO_MIN_TIME;
		hsv.write((uint8_t)(i<<1),(uint16_t)servo[i]);
//		Serial.print_long(servo[i],DEC);
//		Serial.print(",");
	}
	
#ifdef DC2SV1
	if(offset)
	{
		if(abs(fly_param.motor[1])<60) fly_param.motor[1]=0;
	}
	fly_param.motor[1]*=-2;
	hhb.write(2,(long)fly_param.motor[1]);
#endif
	fly_param.motor[2]*=-1;
	hhb.write(3,(long)fly_param.motor[2]);
	hhb.update();
//	Serial.println_long(fly_param.motor[2],DEC);
	
	return;
}

// updating calibration motor
void update_calib_data(void)
{
	if(((short)Flash.read(0,0)!=0x3110) || ((short)Flash.read(0,1) != fly_param.servo_offset[0]) || ((short)Flash.read(0,2) != fly_param.servo_offset[1]))
	{
		Flash.erase(0);
		Flash.write(0,0,0x3110);			// servo_offset_valid
		Flash.write(0,1,fly_param.servo_offset[0]);
		Flash.write(0,2,fly_param.servo_offset[1]);
	#ifdef DEBUG
		Serial.print("Servo offset update");
		Serial.print("\tX=");
		Serial.print_long((long)fly_param.servo_offset[0],DEC);
		Serial.print("\tY=");
		Serial.println_long((long)fly_param.servo_offset[1],DEC);
	#endif
	}
}

// button chattaring
unsigned char button_check(unsigned char button)
{
	//  0 0 1  --> button is pushed
	//  1 0 1  --> chattering
	
	static unsigned char prev_button[2];
	unsigned char result;
	
	result = (unsigned char)(button & ~prev_button[0] & ~prev_button[1]);
	prev_button[1] = prev_button[0];
	prev_button[0] = button;
	return result;
}


//decoder of LED sequencer
void led_update(uint8_t value)
{
	switch(value)
	{
	case LED_OFF:
		digitalWrite(BLUE_LED, HIGH),digitalWrite(ORANGE_LED, HIGH); 
		break;
	case BLUE_ON:
		digitalWrite(BLUE_LED, LOW),digitalWrite(ORANGE_LED, HIGH); 
		break;
	case ORANGE_ON:
		digitalWrite(BLUE_LED, HIGH),digitalWrite(ORANGE_LED, LOW); 
		break;
	case LED_BOTH:
		digitalWrite(BLUE_LED, LOW),digitalWrite(ORANGE_LED, LOW); 
		break;
	default:
		break;
	}
}

// init LED sequencer
void led_init(void)
{
	fly_param.led.state = false;
	fly_param.led.cycle = 0;
	fly_param.led.sequence = NULL;
	fly_param.led.sequence_time = NULL;
	led_update(LED_OFF);
}


// LED sequencer
void led_ctrl(void)
{
	if((fly_param.led.cycle == 0) ||(fly_param.led.sequence == NULL) || (fly_param.led.sequence_time == NULL))  return;
	
	if(fly_param.led.state == false)
	{
		fly_param.led.state = true;
		led_update(*fly_param.led.sequence);
		fly_param.led.event_time = fly_param.current_time;
		return;
	}
	if((fly_param.current_time - fly_param.led.event_time) >= *fly_param.led.sequence_time)
	{
		fly_param.led.cycle--;
		if(fly_param.led.cycle == 0)
		{
			led_init();
		}
		else
		{
			fly_param.led.sequence++;
			fly_param.led.sequence_time++;
			fly_param.led.event_time = fly_param.current_time;
		}
		led_update(*fly_param.led.sequence);
	}
}
void calib_led_func(short value)
{
	int led_mode;
	if(value == 0)
	{
		led_mode=0;
		
	}
	else if(value > 0)
	{
		led_mode = 1;
	}
	else
	{
		led_mode = 2;
	}
	fly_param.led.cycle = led_param[led_mode].cycle;
	fly_param.led.sequence = led_param[led_mode].sequence;
	fly_param.led.sequence_time = led_param[led_mode].sequence_time;
	
	return;
}
void button_func(unsigned char button)
{
	if(button == 0) return;
	
	if(button&0x01)	// increment Y offset 
	{
		fly_param.servo_offset[1] += SERVO_CALIBRATION_UNIT;
		if(fly_param.servo_offset[1]>SERVO_CALIBRATION_MAX) fly_param.servo_offset[1]=SERVO_CALIBRATION_MAX;
		else calib_led_func( fly_param.servo_offset[1]);
#ifdef DEBUG
		Serial.print("Y offset+\t");
		Serial.print_long((long)fly_param.servo_offset[0],DEC);
		Serial.print("\t");
		Serial.println_long((long)fly_param.servo_offset[1],DEC);
#endif
	}
	else if(button&0x02)	// decrement Y offset 
	{
		fly_param.servo_offset[1] -= SERVO_CALIBRATION_UNIT;
		if(fly_param.servo_offset[1]<SERVO_CALIBRATION_MIN) fly_param.servo_offset[1]=SERVO_CALIBRATION_MIN;
		else calib_led_func( fly_param.servo_offset[1]);
#ifdef DEBUG
		Serial.print("Y offset-\t");
		Serial.print_long((long)fly_param.servo_offset[0],DEC);
		Serial.print("\t");
		Serial.println_long((long)fly_param.servo_offset[1],DEC);
#endif
	}
	else if(button&0x04)	// decrement X offset 
	{
		fly_param.servo_offset[0] -= SERVO_CALIBRATION_UNIT;
		if(fly_param.servo_offset[0]<SERVO_CALIBRATION_MIN) fly_param.servo_offset[0]=SERVO_CALIBRATION_MIN;
		else calib_led_func( fly_param.servo_offset[0]);
#ifdef DEBUG
		Serial.print("X offset-\t");
		Serial.print_long((long)fly_param.servo_offset[0],DEC);
		Serial.print("\t");
		Serial.println_long((long)fly_param.servo_offset[1],DEC);
#endif
	}
	else if(button&0x08)	// increment X offset 
	{
		fly_param.servo_offset[0] += SERVO_CALIBRATION_UNIT;
		if(fly_param.servo_offset[0]>SERVO_CALIBRATION_MAX) fly_param.servo_offset[0]=SERVO_CALIBRATION_MAX;
		else calib_led_func( fly_param.servo_offset[0]);
#ifdef DEBUG
		Serial.print("X offset+\t");
		Serial.print_long((long)fly_param.servo_offset[0],DEC);
		Serial.print("\t");
		Serial.println_long((long)fly_param.servo_offset[1],DEC);
#endif
	}
	else if(button&0x10)
	{
	}
	else if(button&0x20)
	{
	}
	else if(button&0x40)
	{
	}
	else if(button&0x80)
	{
	}
	return;
}
/*!
 * @brief about getSensorParam
 * getting sensor parameter
 * 
 * get acc value from sensor
 * cal a and angle
 * @param  none
 * @return none
 */

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
	data = 0x75;
	kxg03.write(KXG03_ACCEL_ODR_WAKE,&data,1);
	
	// set GYRO ODR to 25Hz
	data = 0x05;
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
	rc = kxg03.get_val(val);
	degPitch = atan(val[3] / sqrt(sq(val[4]) + sq(val[5]))) * RAD_TO_DEG;
//	degRoll  = atan(val[5]/  sqrt(sq(val[3]) + sq(val[4]))) * RAD_TO_DEG;
	degRoll  = atan(val[5]/  fabs(val[3])) * RAD_TO_DEG;
//	degRoll  = atan(val[4]/  val[5]) * RAD_TO_DEG;
/*
	val[0] =(val[0] - kxg03_bias[0]);		//gx
	val[1] =(val[1] - kxg03_bias[1]);		//gy
	val[2] =(val[2] - kxg03_bias[2]);		//gz
*/
	kPitch=kalman.cal(&kalman_param[0],degPitch,val[2]);
	kRoll=kalman.cal(&kalman_param[1],degRoll,val[0]);
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
	
	Serial.begin(115200);
	
	
	// ########### initializing LEDs ############
	pinMode(BLUE_LED,OUTPUT);
	pinMode(ORANGE_LED,OUTPUT);
	digitalWrite(BLUE_LED,LOW);
	digitalWrite(ORANGE_LED,LOW);
	
	// ########### initializing motor ############
	init_servo_offset();
	
	// Motor LSI Setting
	digitalWrite(MOTOR_PWM, LOW);
	digitalWrite(MOTOR_PS, HIGH);
	
	pinMode(MOTOR_PWM,OUTPUT);
	pinMode(MOTOR_PS,OUTPUT);
	
	// H-Bridge API initializing
	fly_param.last_wearable_time =0;
	fly_param.motor[0] = 0;
	fly_param.motor[1] = 0;
	fly_param.motor[2] = 0;
	fly_param.motor[3] = 0;

#ifdef DC2SV1
	hhb.init(2,1023);
	hhb.attach(2,16,8);
	hhb.write(3,0L);
	hhb.init(3,1023);
	hhb.attach(3,4,5);
	hhb.attach(3,6,7);
	hhb.write(3,0L);
	// Servo Motor API Initializing
	hsv.init(2, 24000);
	hsv.attach(0,9);
	hsv.attach(1,3);
	hsv.write(1,1);					// does not use this channel
	update_motor_data(true);
	hhb.start(2);
	hhb.start(3);
	hsv.start();
#endif
#ifdef DC1SV2
	hhb.init(3,1023);
	hhb.attach(3,4,5);
	hhb.attach(3,6,7);
	hhb.write(3,0L);
	
	hsv.init(4, 24000);
	hsv.attach(0,16);
	hsv.attach(1,8);
	hsv.attach(2,9);
	hsv.attach(3,3);
	hsv.write(0,map(0,-512,511,500,1500));
	hsv.write(1,1);
	hsv.write(2,map(0,-512,511,500,1500));
	hsv.write(3,1);
	update_motor_data(true);
	hhb.start(3);
	hsv.start();
#endif
	
	// ####### Initializing Sensor ##########
	Wire.begin();
	rc = initSensor();
	
	// ########### initializing Sub-GHz ############
	// Initializing Sub-GHz	
	msg = SubGHz.init();
	
	if(msg != SUBGHZ_OK)
	{
		SubGHz.msgOut(msg);
		while(1){ }
	}

	msg = SubGHz.setAckReq(false);
	SubGHz.setBroadcastEnb(false);
	SubGHz.setMyAddress(1);
	msg = SubGHz.begin(SUBGHZ_CH, SUBGHZ_PANID,  SUBGHZ_100KBPS, SUBGHZ_PWR_20MW);
	if(msg != SUBGHZ_OK)
	{
		SubGHz.msgOut(msg);
		while(1){ }
	}

	Serial.println_long(SubGHz.getMyAddress(),HEX);
	
	// ########### Change Sub-GHz mode ############
	// Change Sub-GHz mode to non-ack(AddrMode=4)
	
	// ########### Enabling RX ############
	msg = SubGHz.rxEnable(NULL);
	if(msg != SUBGHZ_OK)
	{
		SubGHz.msgOut(msg);
		while(1){ }
	}
	
	// ########### set up LED ############
	digitalWrite(ORANGE_LED,LOW),digitalWrite(BLUE_LED,HIGH);
	
	// ########### set dummy data ############
	fly_param.last_recv_time = 0x8000;		// initializing recving time
	fly_param.led.cycle = 0;
	fly_param.led.sequence = NULL;
	fly_param.led.sequence_time = NULL;
	
	// ########### init state ###########
	fly_param.func_mode = FLY_STATE_DETECTING_ZERO;


	// param init
	lkPitch=0;
	dPitch=0;
	avrPitchError=0;

	Serial.println_long(sizeof(fly_param.motor),DEC);
	
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
	short rx_len;
//	t0=millis();	
	// ########### Main task ############

	rx_len = SubGHz.readData(rx_data,sizeof(rx_data));				// 受信データを取得
	fly_param.length = 0;
	memset(fly_param.motor,0,sizeof(fly_param.motor));
	fly_param.current_time = millis();								// 現在時刻を取得

	// ########### get start time ############
	if(rx_len > 0)
	{
		// wearable機器からのデータを受信
		if(rx_data[15]&WEARABLE_FLAG)
		{
			fly_param.length =rx_len;								// 受信データを有効にする
			fly_param.last_recv_time = fly_param.current_time;		// 受信時刻に反映する
			fly_param.last_wearable_time = fly_param.current_time;	// wearable操作の時刻を記憶
			memcpy(fly_param.motor,&rx_data[9],8);					// 受信データを有効にする
		} else
		// コントローラからの信号を受信
		{
			if(fly_param.current_time  - fly_param.last_wearable_time > WEARABLE_FORCE_TIME)
			{
				fly_param.length =rx_len;							// 受信データを有効にする
				fly_param.last_recv_time = fly_param.current_time;	// 受信時刻に反映する
				memcpy(fly_param.motor,&rx_data[9],8);				// 受信データを有効にする
			}
		}
	}
	
	// ########### getSensor ############
	if(kxg03_irq)													// センサーのデータが準備できた時の処理
	{
		t0=millis();
		kxg03_irq=false;
		getSensor();												// センサーデータの取得
		update_sensor_flag=true;									// センサーデータを取得したことを示すフラグをセット
		t1 = millis();
	}
	//#### state machine check
	if(fly_param.func_mode>=STATE_ERROR)
	{
		digitalWrite(25,HIGH);
		digitalWrite(26,HIGH);
		while(1){}
		fly_param.func_mode=0;
	}
	// ### Main Task
	t2=millis();
	fly_param.func_mode = functions[fly_param.func_mode]();			// 制御のタスクを実行
	t3=millis();
	update_sensor_flag = false;										// センサーを使用する処理が完了したらフラグをクリアする
	
	// ### led Task
	led_ctrl();														// LED点滅処理
	// ### get end of process time
	
/*	
	if(fly_param.length > 0)
	{
		Print.init(tx_data,sizeof(tx_data));
		Print.p("STX,");
		Print.f(degPitch,0);
		Print.p(",");
		Print.f(degRoll,0);
		Print.p(",");
		Print.f(kPitch,0);
		Print.p(",");
		Print.f(kRoll,0);
		Print.p(",");
		
		Print.f(val[2],0);
		Print.p(",");
		Print.f(val[0],0);
		Print.p(",");

		Print.f(degPitch,0);
		Print.p(",");
		Print.f(degRoll,0);
		Print.p(",");

		Print.p(",ETX");
		Print.ln();
	// ########### send SensorData ############
//		SubGHz.send(SUBGHZ_PANID,SUBGHZ_TXADDR,tx_data,Print.len(),NULL);	// ロギングデータの送信
	}
*/
/*
	{
		static uint32_t last_print_time = 0;
		if(( fly_param.last_recv_time - last_print_time) > 1000)
		{
			Serial.print(print_mode[fly_param.func_mode]);
			Serial.print("\t");
			Serial.print_double(kPitch,2);
			Serial.print("\t");
			Serial.print_long((long)cal_motor[0],DEC);
			Serial.print("\t");
			Serial.print_long((long)cal_motor[1],DEC);
			Serial.print("\t");
			Serial.print_long((long)cal_motor[2],DEC);
			Serial.print("\t");
			Serial.println_long((long)cal_motor[3],DEC);
			last_print_time = fly_param.last_recv_time;
		}
	}
*/
}

