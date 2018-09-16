
// FLY State machine
typedef enum {
	FLY_STATE_DETECTING_ZERO = 0,
	FLY_STATE_NORMAL,
	FLY_STATE_CALIB,
	FLY_STATE_HOVERING,
	STATE_ERROR
}FLY_STATE;


#define LED_OFF			0
#define BLUE_ON			1
#define ORANGE_ON		2
#define LED_BOTH		BLUE_ON|ORANGE_ON
typedef struct {
	bool state;
	uint16_t cycle;
	uint8_t *sequence;
	uint32_t *sequence_time;
	uint32_t event_time;
} LED_CTRL;

// Lazurite Fly Main Sequence
typedef struct {
	uint16_t func_mode;
	short length;
	short motor[4];
	short servo_offset[2];
	long current_time;
	long last_recv_time;
	long last_wearable_time;
	LED_CTRL led;
//	char sensor_cycle;
//	float a,x,y,z;
} LAZURITE_FLY_PARAM;

void init_servo_offset(void);
void update_motor_data(bool offset);
void update_calib_data(void);

unsigned char button_check(unsigned char button);
void button_func(unsigned char button);

void led_init(void);
void led_ctrl(void);
void calib_led_func(short value);

FLY_STATE func_waiting_zero(void);
FLY_STATE func_normal(void);
FLY_STATE func_motor_callibration(void);
FLY_STATE func_hovering(void);


// state machine of functions
FLY_STATE (*functions[])(void) ={
	func_waiting_zero,
	func_normal,
	func_motor_callibration,
	func_hovering
};

