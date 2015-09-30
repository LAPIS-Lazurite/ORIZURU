/* FILE NAME: Read_SubGHz.c
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
// Sub-GHz
#define SUBGHZ_CH	24
#define SUBGHZ_PANID	0xABCD
#define SUBGHZ_TXADDR	0x87A4
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

// sensor parameters
float   SelfTest[6];
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer

static LAZURITE_FLY_PARAM fly_param;

#ifdef DEBUG
const char print_mode[][16]={
	"Waitint zero",
	"Normal",
	"callibration"
};
#endif

typedef struct {
	uint8_t cycle;
	uint8_t sequence[3];
	uint32_t sequence_time[3];
}LED_PARAM;


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
	
	if((fly_param.length > 0) && (fly_param.motor[2] == 0))
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
			update_motor_data();
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
	
	if(fly_param.length ==0)
	{
		if((fly_param.current_time-fly_param.last_recv_time)>500)
		{
			func_waiting_zero();
			mode = FLY_STATE_DETECTING_ZERO;
		}
	}
	else
	{
		update_motor_data();		//update motor data
		// センサーデータ取得
		// センサーデータの送信
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
	
	return mode;
}

// State of motor callibration
FLY_STATE func_motor_callibration(void)
{
	FLY_STATE mode = FLY_STATE_CALIB;
	
	if(fly_param.length > 0)
	{
		if(fly_param.motor[2] != 0)	// release calibration mode
		{
			update_calib_data();		// 
			mode = FLY_STATE_NORMAL;
			func_normal();
		}
		{							// calibration mode
			button_func(button_check((unsigned char)fly_param.motor[3]));
			update_motor_data();
		}
	}
	else
	{
		if((fly_param.current_time-fly_param.last_recv_time)>500)
		{
			func_waiting_zero();
			mode = FLY_STATE_DETECTING_ZERO;
		}
	}
	
	return mode;
}

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

void update_motor_data(void)
{
	int i;
	long servo[2];
	for(i=0;i<2;i++)
	{
		servo[i] = map((long)fly_param.motor[i],FLY_CTRL_MIN,FLY_CTRL_MAX,SERVO_MIN_TIME,SERVO_MAX_TIME);
		servo[i] += fly_param.servo_offset[i];
		if(servo[i] >= SERVO_MAX_TIME) servo[i] = SERVO_MAX_TIME;
		else if (servo[i] <= SERVO_MIN_TIME) servo[i] = SERVO_MIN_TIME;
		hsv.write((uint8_t)(i<<1),(uint16_t)servo[i]);
	}
	hhb.write(3,(long)fly_param.motor[2]);
	hhb.update();
#ifdef DEBUG
	if(fly_param.func_mode == FLY_STATE_NORMAL)
	{
		Serial.print("Motor\t");
		Serial.print_long(servo[0],DEC);
		Serial.print("\t");
		Serial.print_long(servo[1],DEC);
		Serial.print("\t");
		Serial.println_long((long)fly_param.motor[2],DEC);
	}
#endif
	
	return;
}

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

void led_init(void)
{
	fly_param.led.state = false;
	fly_param.led.cycle = 0;
	fly_param.led.sequence = NULL;
	fly_param.led.sequence_time = NULL;
	led_update(LED_OFF);
}



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

void setup(void)
{
	SUBGHZ_MSG msg;
	long myAddress;
	SUBGHZ_PARAM param;
	
#ifdef DEBUG
	Serial.begin(115200);
#endif
	
	
	// ########### initializing LEDs ############
	pinMode(BLUE_LED,OUTPUT);
	pinMode(ORANGE_LED,OUTPUT);
	digitalWrite(BLUE_LED,LOW);
	digitalWrite(ORANGE_LED,LOW);
	
#ifdef SENSOR_LOGGER
	// ########### initializing Gyro ############
	// I2C Initializing
	Wire.begin();
	// Gyro Test
	MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
	Serial.print("x-axis self test: acceleration trim within : "); Serial.print_double((double)SelfTest[0],1); Serial.println("% of factory value");
	Serial.print("y-axis self test: acceleration trim within : "); Serial.print_double((double)SelfTest[1],1); Serial.println("% of factory value");
	Serial.print("z-axis self test: acceleration trim within : "); Serial.print_double((double)SelfTest[2],1); Serial.println("% of factory value");
	Serial.print("x-axis self test: gyration trim within : "); Serial.print_double((double)SelfTest[3],1); Serial.println("% of factory value");
	Serial.print("y-axis self test: gyration trim within 	: "); Serial.print_double((double)SelfTest[4],1); Serial.println("% of factory value");
	Serial.print("z-axis self test: gyration trim within : "); Serial.print_double((double)SelfTest[5],1); Serial.println("% of factory value");
	delay(5000);
	
	calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
	
	delay(1000); 
	
	initMPU9250(); 
#endif
	// ########### initializing motor ############
	init_servo_offset();
	
	// Motor LSI Setting
	digitalWrite(MOTOR_PWM, LOW);
	digitalWrite(MOTOR_PS, HIGH);
	
	pinMode(MOTOR_PWM,OUTPUT);
	pinMode(MOTOR_PS,OUTPUT);
	
	// H-Bridge API initializing
	hhb.init(3,1023);
	hhb.attach(3,4,5);
	hhb.attach(3,6,7);
	hhb.write(3,0L);
	
	// Servo Motor API Initializing
	hsv.init(4, 24000);
	hsv.attach(0,9);
	hsv.attach(1,3);
	hsv.attach(2,16);
	hsv.attach(3,20);
	hsv.write(1,1);					// does not use this channel
	hsv.write(3,1);					// does not use this channel
	fly_param.motor[0] = 0;
	fly_param.motor[1] = 0;
	fly_param.motor[2] = 0;
	fly_param.motor[3] = 0;
	update_motor_data();
	hhb.start(3);
	hsv.start();
	
	// ########### initializing Sub-GHz ############
	// Initializing Sub-GHz	
	msg = SubGHz.init();
	
	if(msg != SUBGHZ_OK)
	{
		SubGHz.msgOut(msg);
		while(1){ }
	}
#ifdef DEBUG
	myAddress = SubGHz.getMyAddress();
	Serial.print("myAddress1 = ");
	Serial.println_long(myAddress,HEX);	
#endif
	msg = SubGHz.begin(SUBGHZ_CH, SUBGHZ_PANID,  SUBGHZ_100KBPS, SUBGHZ_PWR_20MW);
	if(msg != SUBGHZ_OK)
	{
		SubGHz.msgOut(msg);
		while(1){ }
	}
	
	// ########### initializing Graph tool in Raspberry Pi ############
	// Reset parameters
	Print.init(tx_data,sizeof(tx_data));
	Print.p("SensorReset,");
	SubGHz.send(SUBGHZ_PANID,SUBGHZ_TXADDR, (unsigned char *)&tx_data, Print.len(),NULL);
	sleep(10);

	// Send Sensor Name and dimention of data
	Print.init(tx_data,sizeof(tx_data));
	Print.p("SensorList,Gyro,3,");
	SubGHz.send(SUBGHZ_PANID,SUBGHZ_TXADDR, (unsigned char *)&tx_data, Print.len(),NULL);
	
	// ########### Change Sub-GHz mode ############
	// Change Sub-GHz mode to non-ack(AddrMode=4)
	SubGHz.getSendMode(&param);
	param.addrType = 4;
	SubGHz.setSendMode(&param);
	
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
	return;
}

void loop(void)
{
	// ########### Main task ############
	fly_param.length = SubGHz.readData(rx_data,sizeof(rx_data));
	fly_param.current_time = millis();
	if(fly_param.length > 0)
	{
		fly_param.last_recv_time = fly_param.current_time;
		memcpy(fly_param.motor,&rx_data[7],8);
	}
	
	if((fly_param.func_mode>=STATE_ERROR) || (fly_param.func_mode < FLY_STATE_DETECTING_ZERO))
	{
		while(1){}				// error state
	}
	else
	{
		fly_param.func_mode = functions[fly_param.func_mode]();
	}
	
#ifdef DEBUG
	{
		static uint32_t last_print_time = 0;
		if(( fly_param.current_time - last_print_time) > 1000)
		{
			Serial.print(print_mode[fly_param.func_mode]);
			Serial.print(",");
			Serial.print_long(fly_param.current_time,DEC);
			Serial.print("\t");
			Serial.print_long(last_print_time,DEC);
			Serial.print("\t");
			Serial.print_long((long)fly_param.motor[0],DEC);
			Serial.print("\t");
			Serial.print_long((long)fly_param.motor[1],DEC);
			Serial.print("\t");
			Serial.print_long((long)fly_param.motor[2],DEC);
			Serial.print("\t");
			Serial.println_long((long)fly_param.motor[3],DEC);
			last_print_time = fly_param.current_time;
		}
	}
#endif
	led_ctrl();
}

