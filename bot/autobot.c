/*******************************************************************************
* autobot.c
*
*******************************************************************************/


/**
need to do:
1. dynamica model, measure, base length
2. sensor fusion
3. paper
4. communication with RPi

**/

// usefulincludes is a collection of common system includes for the lazy
// This is not necessary for roboticscape projects but here for convenience
#include "../robocape/libraries/rc_usefulincludes.h"//<rc_usefulincludes.h> 
// main roboticscape API header
#include "../robocape/libraries/roboticscape.h"//<roboticscape.h>
#include "../lcmtypes/robot_path_t.h"
#include "../lcmtypes/pose_xyt_t.h"


#define 	SAMPLE_RATE_HZ 			200	  // main filter and control loop speed
#define 	SAMPLE_DELTA_T 			1.0/(float)SAMPLE_RATE_HZ // 1/sample_rate
#define		PRINTF_HZ				20  // frequency for print thread
// choice of 1,2,4,8,16 oversampling. Here we use 16 and sample at 25hz which
// is close to the update rate specified in robotics_cape.h for that oversample.
#define OVERSAMPLE  BMP_OVERSAMPLE_4
// choice of OFF, 2, 4, 8, 16 filter constants. Here we turn off the filter and 
// opt to use our own 2nd order filter instead.
#define INTERNAL_FILTER	BMP_FILTER_OFF

// our own low pass filter
#define ORDER			2
#define CUTOFF_FREQ		2.0f	// 2rad/s, about 0.3hz
#define BMP_CHECK_HZ	100		//BMP frequency is 100Hz
#define	DT_BMP			1.0f/BMP_CHECK_HZ

//encoder channel
#define encoder_chan_1 1
#define encoder_chan_2 2

//wheel, need measure
#define D 0.0665f //wheel diameter in meter
#define GEARBOX 4 //gear ratio 30? 4?      check the purchase history, gear ratio changed???
#define ENCODER_RES 390 //encoder rev ?? 
#define	WHEEL_BASE				0.1675
#define ENCODER_RATIO (PI*D)/(GEARBOX*ENCODER_RES)

//path
#define ERR_TOL_DIST 0.09//0.02//0.09
#define ERR_TOL_THETA 0.05//0.02//0.05

//odometry
#define ODOM_GYRO_THRESHOLD 0.000023
#define BOT_MOTION_TURN_THRESHOLD 8 /////////////////

//GPS UART configuration
#define GPS_BUF_SIZE 	64
#define TIMEOUT_S 	1.5
#define BAUDRATE 	9600
#define GPS_BUS 5

char gps_buf[GPS_BUF_SIZE]; 
int ret; // return value
int GPS_bytes = 1;



int odom_count = 0;
int control_count = 0;
int rotate_count = 0;



//test
float target_x = 0;
float target_y = 0;
float target_err = 0;



// data type declaration
rc_imu_data_t imu_data;


// set button usage
int record_flag = 0;

//motor
#define MOTOR_PWM_FREQ 2000

typedef struct{
    int pwm_num;
    char pwm_chan;
    int dir_1;
    int dir_2;
    float speed;
} motor_t;

motor_t motor_1;
motor_t motor_2;

typedef struct world_odometry//this is for odometry
{
	float x;
	float y;
	float theta;
	float prev_encoder_val_1;
	float prev_encoder_val_2;
	float prev_theta;
}world_odometry;

typedef struct bot_data_t// bot kinematic state
{
	//current pos values	
	float accel_x;
	float accel_y;	
	float accel_z;

	float gyro_x;
	float gyro_y;	
	float gyro_z;	

	float roll;
	float pitch;	
	float yaw;
	float prev_yaw;//calculate delta_theta_gyro

	float pitch_speed;
	float yaw_speed;
	float roll_speed;

	//forward and angular velocity
	float set_forward_vel;
	float set_angular_vel;
	// feedback velocity
	float fb_forward_vel;
	float fb_angular_vel;
	float fb_wheel_vel_1;
	float fb_wheel_vel_2;
	//error
	float delta_theta_gyro;
	float delta_theta_encoder;

	float imu_temp;//may not use

	float temp;
	float pressure;
	float altitude;
	float filtered_alt;

	float GPS_time_temp;//GPS time
	float GPS_N;//north latitude
	float GPS_W;//west longitude
	float GPS_alti;//altitude of the antenna
	float GPS_geo;//geoidal separation 

} bot_data_t;

typedef enum bot_motion_state_t {
	IDLE,
	READY,
	TURN,
	RUN,
	RC,
} bot_motion_state_t;

typedef struct PID//this is for PID
{
	float kp;
	float ki;
	float kd;
	float last_err;
	float accumulate; //for I: accu = accu + ki*err*DT

} PID;

world_odometry world_odom = {
	.x = 0,
	.y = 0,
	.prev_encoder_val_1 = 0,
	.prev_encoder_val_2 = 0,
	.theta = 0,
	.prev_theta = 0,
};
bot_data_t bot_data = {
	//forward and angular velocity
	.set_forward_vel = 0,
	.set_angular_vel = 0,
};
//******************************tune
PID pathPID = {
	.kp = 3,//4
	.ki = 0,
	.kd = 0.03,//0.01
	.last_err = 0,
	.accumulate = 0,
};

bot_motion_state_t bot_motion_state = IDLE;

//filters
rc_filter_t lowpass_alti;
rc_filter_t velocity_filter;
rc_filter_t angular_filter;
rc_filter_t delta_theta_gyro_filter;
rc_filter_t delta_theta_odom_filter;

// function declarations
void init_parameters();
void on_pause_pressed();
float theta_regulate(float theta);
float velocity_regulate(float vel);
void cascade_control();
void odometry_get(int freq_div);
void bot_motion_control(int freq_div);
//void on_pause_released();
void function_start_green();
void function_end_green();
void interrupt_IMU_function();
void imu_get(); // imu get function
void barometer_get();
void print_header();
//threads
void* printf_loop(void* ptr);
void* GPS_loop(void* ptr);
void* lcm_publish_loop(void* ptr);
void robot_path_handler(const lcm_recv_buf_t *rbuf, const char * channel, const robot_path_t * msg, void * user);

//motor
void setup_motors();
motor_t init_motor(int pwm_num, char pwm_chan, int dir_pin_1, int dir_pin_2);
int set_motor_speed(motor_t* motor, float speed);
int uninit_motor(motor_t* motor);

//path calculation
float heading_pose_botgui(pose_xyt_t target_pose, world_odometry bot_pose);

//PID
float PID_Cal(PID* pid, float err, float DT);


//LCM
robot_path_t *path_list;
// LCM Global Variables
lcm_t * lcm;
static const char ODOMETRY_CHANNEL[] = "ODOMETRY_POSE"; //odometry channel
static const char ROBOT_PATH_T_CHANNEL[] = "CONTROLLER_PATH"; //robot_path_t channel
static const char TRUEPOSE_CHANNEL[] = "TRUE_POSE"; //robot_path_t channel
pose_xyt_t pose_xyz = {
	.utime = 0,
	.x = 0,
	.y = 0,
	.theta = 0,
};


//for path planning, parameters defined as below
unsigned int path_idx;
float path_list_origin[2];

/*******************************************************************************
* int main() 
*
* This template main function contains these critical components
* - call to rc_initialize() at the beginning
* - main while loop that checks for EXITING condition
* - rc_cleanup() at the end
*******************************************************************************/
int main(){
	// always initialize cape library first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
		return -1;
	}
	rc_set_cpu_freq(FREQ_1000MHZ);
	init_parameters();
	lcm = lcm_create(NULL);
	rc_set_pinmux_mode(GPS_HEADER_PIN_3, PINMUX_PWM);
	rc_set_pinmux_mode(GPS_HEADER_PIN_4, PINMUX_PWM);

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	if(isatty(fileno(stdout))){
		pthread_t  printf_thread;
		pthread_create(&printf_thread, NULL, printf_loop, (void*) NULL);
	}

/*	//GPS initialization

	if(rc_uart_init(GPS_BUS, BAUDRATE, TIMEOUT_S)){
		printf("Failed to rc_uart_init%d\n", GPS_BUS);
		rc_cleanup();
		return -1;
	}
	//GPS thread
	pthread_t  GPS_thread;
	pthread_create(&GPS_thread, NULL, GPS_loop, (void*) NULL);
*/

	// start with default config and modify based on options
	rc_imu_config_t imu_config = rc_default_imu_config();
	imu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	imu_config.show_warnings = 1;
	// imu_config.enable_magnetometer=1; // not very accurate, don't use indoor
	//imu_config.orientation = ORIENTATION_Y_UP;

/*	// initialize barometer
	if(rc_initialize_barometer(OVERSAMPLE, INTERNAL_FILTER)<0){
		fprintf(stderr,"ERROR: rc_initialize_barometer failed\n");
		return -1;
	}
	double altitude;
	lowpass_alti = rc_empty_filter();
	// create the lowpass filter and prefill with current altitude
	if(rc_butterworth_lowpass(&lowpass_alti,ORDER, DT, CUTOFF_FREQ)){
		fprintf(stderr,"ERROR: failed to make butterworth filter\n");
		return -1;
	}
	altitude = rc_bmp_get_altitude_m();
	rc_prefill_filter_inputs(&lowpass_alti, altitude);
	rc_prefill_filter_outputs(&lowpass_alti, altitude);

*/
	//start lcm publish thread
	pthread_t  lcm_publish_thread;
	pthread_create(&lcm_publish_thread, NULL, lcm_publish_loop, (void*) NULL);

	// now set up the imu for dmp interrupt operation
	if(rc_initialize_imu_dmp(&imu_data, imu_config)){
		printf("rc_initialize_imu_failed\n");
		return -1;
	}

	// turn on green light
	function_start_green();

	// button thread
	rc_set_pause_pressed_func(&on_pause_pressed);
	printf("Pause button interrupt starts.\n");
	//rc_set_pause_released_func(&on_pause_released);

	// LCM thread: receive robot_path_t channel
	robot_path_t_subscribe(lcm, ROBOT_PATH_T_CHANNEL, &robot_path_handler, NULL);

	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 

	// motor initialization
	setup_motors();

	// write labels for what data will be printed and associate the interrupt
	// function to print data immediately after the header.
	rc_set_imu_interrupt_func(&interrupt_IMU_function);
	//printf("IMU interrupt starts.\n");

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){
		// always sleep at some point
		// usleep(100000);
		lcm_handle_timeout(lcm,10000);
	}

	lcm_destroy(lcm);
	//GPS UART clean
	rc_uart_close(GPS_BUS);
	//close motor
	uninit_motor(&motor_1);
	uninit_motor(&motor_2);
	//clean roboticscape
	rc_power_off_imu();
	// turn off green light
	function_end_green();
	// exit cleanly
	rc_cleanup(); 
	return 0;
}

/*******************************************************************************
* robot_path_handler()
*
*******************************************************************************/

void robot_path_handler(const lcm_recv_buf_t *rbuf, const char * channel, const robot_path_t * msg, void * user){
/*
	if(bot_motion_state != IDLE)
		return;*/
	int i;
	//printf("Receive LCM msg from channel CONTROLLER_PATH.\n");
	for(i=0;i<msg->path_length;i++){
		printf("\nPoint index No.: %d, x: %6.3f, y: %6.3f\n", i, msg->path[i].x, msg->path[i].y);
	}
	path_list = robot_path_t_copy(msg);// copy const robot_path_t * msg to global pointer path_list
/*	if(msg->path_length < 2)
		return;*/

	path_list_origin[0] = world_odom.x;
	path_list_origin[1] = world_odom.y;
	bot_motion_state = RUN;// set up state of bot motion, enum bot_motion_state_t
	path_idx = 0;// loop from the first/second point until the last point
}


/*******************************************************************************
* setup_motors()
*
*******************************************************************************/
void setup_motors(){
// TODO: INITIALIZE THE MOTORS
// use set_encoder_pos and init_motor commands
	//init_motor, depend on the wiring, from test_motor.c
	printf("Set up motors 1,2.\n");
	motor_1 = init_motor(0,'A',113,116);
	motor_2 = init_motor(0,'B',49,57);
	//motor econder pos init both to 0;
	rc_set_encoder_pos(encoder_chan_1, 0);
	rc_set_encoder_pos(encoder_chan_2, 0);
}


//initialize motor
motor_t init_motor(int pwm_num, char pwm_chan, int dir_pin_1, int dir_pin_2){
	// Initialize motor by setting up GPIO and PWM
	motor_t motor;
	motor.pwm_num = pwm_num;
	motor.pwm_chan = pwm_chan;
	// Initialize motor direction control, here is for 298 motor control
	motor.dir_1 = dir_pin_1;
	motor.dir_2 = dir_pin_2;

	// export the direction pin and enable pin for GPIO
	//The roboticscape service will export all available GPIO pins on boot so exporting manually is not strictly necessary, 
	//but must still configure the pin and an input or output as you desire.
	rc_gpio_export(dir_pin_1);
	rc_gpio_export(dir_pin_2);
	// Set the GPIO pins to either input or output
	rc_gpio_set_dir(dir_pin_1, OUTPUT_PIN);
	rc_gpio_set_dir(dir_pin_2, OUTPUT_PIN);

	// motor direction initialization
	rc_gpio_set_value_mmap(dir_pin_1, LOW);
	rc_gpio_set_value_mmap(dir_pin_2, LOW);

	//PWM
	rc_pwm_init(pwm_num, MOTOR_PWM_FREQ);
	return motor;
}

//set pwm value/ motor speed
int set_motor_speed(motor_t* motor, float speed){
	// Adjust motor speed via PWM and direction via the dir pins
	// SV1, SV3: are the actual pin locations on the board correspond to
	//           A0, A1. 
	if(speed >= 0){
		rc_gpio_set_value_mmap(motor->dir_1, LOW);
		rc_gpio_set_value_mmap(motor->dir_2, HIGH);
	}else if(speed < 0){
		rc_gpio_set_value_mmap(motor->dir_1, HIGH);
		rc_gpio_set_value_mmap(motor->dir_2, LOW);		
	}
/*	if(speed > 0){
		rc_gpio_set_value_mmap(motor->dir_1, LOW);
		rc_gpio_set_value_mmap(motor->dir_2, HIGH);
	}else if(speed < 0){
		rc_gpio_set_value_mmap(motor->dir_1, HIGH);
		rc_gpio_set_value_mmap(motor->dir_2, LOW);		
	}else{
		rc_gpio_set_value_mmap(motor->dir_1, LOW);
		rc_gpio_set_value_mmap(motor->dir_2, LOW);		
	}*/

	rc_pwm_set_duty_mmap(motor->pwm_num, motor->pwm_chan, fabs(speed));
	return 1;
}

//uninitialize motor
int uninit_motor(motor_t* motor){
	// Unitialize motor by disableing the motor, and uninitializing PWMs
	printf("Uninitializa motor.\n");
	// GPIO
	rc_gpio_set_value_mmap(motor->dir_1, LOW);
	rc_gpio_set_value_mmap(motor->dir_2, LOW);

	rc_gpio_unexport(motor->dir_1);
	rc_gpio_unexport(motor->dir_2);

	// PWM
	rc_pwm_close(motor->pwm_num);

	return 0;
}


// initial parameters
void init_parameters(){
	printf("Initialize parameters.\n");
	//rc_first_order_lowpass(&velocity_filter, dt, TIME_CONSTANT);
	velocity_filter = rc_empty_filter();
	angular_filter = rc_empty_filter();
	delta_theta_gyro_filter = rc_empty_filter();
	delta_theta_odom_filter = rc_empty_filter();
/*	rc_first_order_lowpass(&velocity_filter, SAMPLE_DELTA_T*1.0, SAMPLE_DELTA_T*80.0);
	rc_first_order_lowpass(&angular_filter, SAMPLE_DELTA_T*1.0, SAMPLE_DELTA_T*80.0);
	rc_first_order_lowpass(&delta_theta_gyro_filter, SAMPLE_DELTA_T*1.0, SAMPLE_DELTA_T*80.0);
	rc_first_order_lowpass(&delta_theta_odom_filter, SAMPLE_DELTA_T*1.0, SAMPLE_DELTA_T*80.0);*/
	rc_first_order_lowpass(&velocity_filter, SAMPLE_DELTA_T*1.0, SAMPLE_DELTA_T*1.0);
	rc_first_order_lowpass(&angular_filter, SAMPLE_DELTA_T*1.0, SAMPLE_DELTA_T*1.0);
	rc_first_order_lowpass(&delta_theta_gyro_filter, SAMPLE_DELTA_T*1.0, SAMPLE_DELTA_T*1.0);
	rc_first_order_lowpass(&delta_theta_odom_filter, SAMPLE_DELTA_T*1.0, SAMPLE_DELTA_T*1.0);

}

// green light on function
void function_start_green(){
	rc_set_led(GREEN, ON);
	printf("Green light indicater on.\n");
	return;
}

// green light off function
void function_end_green(){
	rc_set_led(GREEN,OFF);
	printf("Green light indicater off.\n");
	return;
}

/*******************************************************************************
* void print_header()
*
* Based on which data is marked to be printed, print the correct labels.
* this is printed only once and the actual data is updated on the next line.
*******************************************************************************/
void print_header(){
	printf(" ");
/*	printf("    DMP Quaternion   |");*/
	printf(" DMP TaitBryan (radians) |");
	printf("   Accel XYZ (g)   |");
	printf("  Gyro XYZ (deg/s) |");
/*	printf(" Temp(C)");
	printf("  temp  |");
	printf(" pressure  |");
	printf(" altitude |");
	printf(" filtered |");
	printf(" GPS_time|");
	printf(" GPS_N|");
	printf(" GPS_W|");*/	
	printf("\n");
}

/*******************************************************************************
* printf_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*******************************************************************************/
void* printf_loop(void* ptr){
	printf("Print thread starts.\n");
	rc_state_t prev_flow_state, cur_flow_state;
	while(rc_get_state()!=EXITING){
		cur_flow_state = rc_get_state();
		if(cur_flow_state==RUNNING && prev_flow_state!=RUNNING){
			print_header();
		}
		else if(cur_flow_state==PAUSED && prev_flow_state!=PAUSED){
			printf("\nPAUSED\n");
		}
		prev_flow_state = cur_flow_state;
		if(cur_flow_state == RUNNING){	
			printf("\r");
			printf(" ");

/*			// print quaternion
			printf(" %4.1f %4.1f %4.1f %4.1f |",	imu_data.dmp_quat[QUAT_W], \
														imu_data.dmp_quat[QUAT_X], \
														imu_data.dmp_quat[QUAT_Y], \
														imu_data.dmp_quat[QUAT_Z]);
*/
			// print TaitBryan angles
			printf("%6.2f %6.2f %6.4f |",	bot_data.roll,bot_data.pitch,bot_data.yaw);
			printf(" %5.2f %5.2f %5.2f |",	bot_data.accel_x,bot_data.accel_y,bot_data.accel_z);
			printf(" %5.1f %5.1f %5.1f |",	bot_data.gyro_x,bot_data.gyro_y,bot_data.gyro_z);
			
			//printf("target_x %5.3f target_y %5.3f pose_x %5.3f pose_y %5.3f error: %5.3f|",	target_x,target_y,pose_xyz.x,pose_xyz.y,target_err);
			printf("pose_x %5.3f pose_y %5.3f theta %5.4f|",world_odom.x,world_odom.y, world_odom.theta);
			printf("gyro %f encoder %f|",bot_data.delta_theta_gyro,bot_data.delta_theta_encoder);
/*			printf("%5.3f |", bot_data.imu_temp);
			printf("%6.2fC |", bot_data.temp);
			printf("%7.2fkpa |", bot_data.pressure/1000.0);
			printf("%8.2fm |", bot_data.altitude);
			printf("%8.2fm |", bot_data.filtered_alt);
			printf("%f |", bot_data.GPS_time_temp);
			printf("%f |", bot_data.GPS_N);
			printf("%f |", bot_data.GPS_W);*/
			fflush(stdout);
		}
		usleep(1000000 / PRINTF_HZ);
	}
	printf("Print thread ends.\n");
	return NULL;
} 



void interrupt_IMU_function(){
	//imu_interrupt_function();
	imu_get(1);
	//barometer_get();
	cascade_control();

}


void cascade_control(){
	odom_count+=1;
	if(odom_count == 1){
		odom_count = 0;
		odometry_get(1);		
	}
	control_count += 1;
	if(control_count == 1){
		//RC_Control(15);
		bot_motion_control(1);
		control_count = 0;
	}

	float Motor_1_Speed = (bot_data.set_forward_vel ) * 1 - bot_data.set_angular_vel * WHEEL_BASE / 2.0;
	float Motor_2_Speed = (bot_data.set_forward_vel) * 1 + bot_data.set_angular_vel * WHEEL_BASE / 2.0;

	Motor_1_Speed = velocity_regulate(Motor_1_Speed*2.5);
	Motor_2_Speed = velocity_regulate(Motor_2_Speed*2.5);	

	set_motor_speed(&motor_1, Motor_1_Speed);
	set_motor_speed(&motor_2, Motor_2_Speed);
}



void bot_motion_control(int freq_div){
	if(bot_motion_state == IDLE){
		return;
	}
/*	if(bot_motion_state == RC){
		return;
	}*/
	// if(path_control(path_list->path[path_idx], path_list->path[path_idx+1], freq_div)){
	// 	path_idx += 1;
	// 	if(path_idx == path_list->path_length - 1){
	// 		bot_motion_state = IDLE;
	// 	}
	// }

	float wx = world_odom.x;
	float wy = world_odom.y;

	//get target pos from command
	float cx = path_list->path[path_idx].x;
	float cy = path_list->path[path_idx].y;


	float err_dist = sqrt((cx - wx)*(cx - wx)   + (cy - wy)*(cy - wy));
	
	if(err_dist < ERR_TOL_DIST ){
		//printf("\nerr_dist < ERR_TOL_DIST.\n");
		//if(fabs(bot_data.fb_forward_vel) < 0.03){
		rotate_count += 1;
		if(rotate_count == 5){
			rotate_count = 0;
			path_idx += 1;
			printf("\nPoint %d [%f,%f] end.\n",(path_idx-1),path_list->path[path_idx-1].x,path_list->path[path_idx-1].y);
			printf("\nDistance error is %f.\n",err_dist);
			printf("\npath ifx %d path_list->path_length %d.\n",path_idx,path_list->path_length);
			//bot_motion_state = TURN;
/*			float theta_temp = atan2(path_list->path[path_idx].y - world_odom.y, path_list->path[path_idx].x - world_odom.x) - world_odom.theta;
			if (fabs(theta_temp)>BOT_MOTION_TURN_THRESHOLD){
				printf("\ntheta_temp:%d,%f,%f\n",path_idx, atan2(path_list->path[path_idx].y - world_odom.y, path_list->path[path_idx].x),world_odom.theta);
				printf("\ntheta_temp:%f\n",theta_temp);
				printf("\nStart turn. \n");
				bot_motion_state = TURN;//consider target point arrived
			}*/
			if(path_idx >= path_list->path_length){
				bot_motion_state = IDLE;
				set_motor_speed(&motor_1,0);
				set_motor_speed(&motor_2,0);
				bot_data.set_forward_vel = 0;
				bot_data.set_angular_vel = 0;
				pathPID.last_err = 0;
				pathPID.accumulate = 0;
				printf("\nIDLE\n");
				return;
			}
			else{
				printf("\nPoint %d [%f,%f] begins.\n",path_idx,path_list->path[path_idx].x,path_list->path[path_idx].y);
			}
		}
		//}
	}
	//run the robot to desired pos and direction
	else{

/*		if( bot_motion_state == TURN){
			printf("\nturn\n");
			// printf(" start turn : wx:%6.2f  \n",wx);
			bot_data.set_forward_vel = 0;
			float theta_err = heading_pose_botgui(path_list->path[path_idx], world_odom);//cur_path->path[path_idx].theta;
			if(fabs(theta_err) < ERR_TOL_THETA){
				bot_data.set_angular_vel = 0;
				printf(" \n***finish Turn : path_idx:%d  \n",path_idx);
				bot_motion_state = RUN;
			}
			else{
				bot_data.set_angular_vel = theta_err<0 ? -3.5:-3.5;//4 : 4;for odom//1:1 for bot caster
			}
		}*/
		if( bot_motion_state == RUN){
			//the velocity can be otherwise calculated by defining a velocity profile
			
			bot_data.set_forward_vel = 0.25;
			//calculate if the robot is right on the path
			float px,py;
			if(path_idx == 0){
				px = path_list_origin[0];
				py = path_list_origin[1];
			}else{
				px = path_list->path[path_idx - 1].x;
				py = path_list->path[path_idx - 1].y;				
			}

			float dir = (cx - px) * (wy - py) - (cy - py) * (wx - px);

			float dir_theta = 1.5;

			//result of atan func: -pi ~ pi. Dangerous range (pi + 1 = - pi -1)
			float target_theta = atan2(cy - py, cx - px) - dir_theta * dir;
			//two method to make bot run on the line
			//PID method
			bot_data.set_angular_vel = PID_Cal(&pathPID, -theta_regulate(target_theta - world_odom.theta),freq_div);
		}
	}
}

/*******************************************************************************
*odometry_get()
*
updte the forward and angular velocity and odometry x,y,theta
*******************************************************************************/

void odometry_get(int freq_div){
    // Read encoder, get difference data of two wheels in certain sampling time interval
	float cur_encoder_val_1 = (float)rc_get_encoder_pos(encoder_chan_1);
	float cur_encoder_val_2 = (float)rc_get_encoder_pos(encoder_chan_2);
	float diff_encoder_val_1 = cur_encoder_val_1 - world_odom.prev_encoder_val_1;
	float diff_encoder_val_2 = cur_encoder_val_2 - world_odom.prev_encoder_val_2;
	world_odom.prev_encoder_val_1 = cur_encoder_val_1;
	world_odom.prev_encoder_val_2 = cur_encoder_val_2;

    float sample_odom_Hz = SAMPLE_RATE_HZ/freq_div;

	//get encoder data that is difference of urrent and previous state
	world_odom.prev_theta = world_odom.theta;

	float d_1 = ENCODER_RATIO*diff_encoder_val_1;
	float d_2 = ENCODER_RATIO*diff_encoder_val_2;
	float delta_yr = 0;//no slide slip, extention left for next step.
	float delta_xr = (d_2 + d_1)/2.0; //distance travelled by robot
	float delta_theta_encoder = (d_1 - d_2)/WHEEL_BASE; //change in heading
	float forward_vel = delta_xr*sample_odom_Hz;
	float angular_vel = delta_theta_encoder*sample_odom_Hz;

	// update data in bot_state (global structure variable)
	//vanilla one
/*	bot_state.fb_forward_vel = forward_vel;
	bot_state.fb_angular_vel = angular_vel;*/

	//filter one
	bot_data.fb_forward_vel = rc_march_filter(&velocity_filter, forward_vel);
	bot_data.fb_angular_vel = rc_march_filter(&angular_filter, angular_vel);

	bot_data.fb_wheel_vel_1 = d_1 * sample_odom_Hz;
	bot_data.fb_wheel_vel_2 = d_2 * sample_odom_Hz;

	//bot_state.delta_theta = delta_theta;

	float delta_theta_gyro = theta_regulate(theta_regulate(bot_data.yaw) - bot_data.prev_yaw);// notice:delta_theta and delta_theta_gyro have different signal
	bot_data.prev_yaw = theta_regulate(bot_data.yaw);

	bot_data.delta_theta_gyro = rc_march_filter(&delta_theta_gyro_filter, delta_theta_gyro);
	bot_data.delta_theta_encoder = rc_march_filter(&delta_theta_odom_filter, delta_theta_encoder);

//	printf("delta gyro %f, encoder %f\n",bot_data.delta_theta_gyro,bot_data.delta_theta_encoder);

/*	float diff_gyro_odom = theta_regulate(theta_regulate(bot_data.delta_theta_gyro) - theta_regulate(bot_data.delta_theta));
*/	if (fabs(theta_regulate(theta_regulate(bot_data.delta_theta_gyro) - theta_regulate(bot_data.delta_theta_encoder))) > ODOM_GYRO_THRESHOLD)
		world_odom.theta = theta_regulate(world_odom.theta + bot_data.delta_theta_gyro);
	else
		world_odom.theta = theta_regulate(world_odom.theta + bot_data.delta_theta_encoder);



    // pure gyro
/*	world_odom.theta = theta_regulate(world_odom.theta + bot_data.delta_theta_gyro);//original
*/
/*	// pure encoder
	world_odom.theta = theta_regulate(world_odom.theta + bot_data.delta_theta_encoder);*/

	//updating world state 	
	world_odom.x = world_odom.x + delta_xr*cos(world_odom.theta) - delta_yr*sin(world_odom.theta);
	world_odom.y = world_odom.y + delta_xr*sin(world_odom.theta) - delta_yr*cos(world_odom.theta);

	// publishing lcm commands assigning theta after gryrodometry
 	pose_xyz.utime = rc_nanos_since_epoch();
	pose_xyz.x = world_odom.x;
	pose_xyz.y = world_odom.y;
	pose_xyz.theta = world_odom.theta;
}
/*******************************************************************************
* theta_regulate()
* check angle to make it range -Pi ~ +Pi
*******************************************************************************/
float theta_regulate(float theta){
	
    while ((theta < -PI)||(theta > PI)){
    	if (theta < -PI)
    		theta = theta + PI*2;
    	if (theta > PI)
    		theta = theta - PI*2;
    }
    return theta;
}
/*******************************************************************************
* velocity_regulate()
* check forward velocity to make it range -1 ~ +1
*******************************************************************************/
float velocity_regulate(float vel){

	if (vel < -0.99)
		vel = -0.99;
	if (vel > 0.99)
		vel = 0.99;
	// if (fabs(vel) < 0.05 && vel>0)
	// 	vel = 0.05;
	// if (fabs(vel) < 0.05 && vel<0)
	// 	vel = -0.05;
    return vel;
}
/*******************************************************************************
* PID_Cal()
*******************************************************************************/
float PID_Cal(PID* pid, float err, float DT){
	pid->accumulate += err*DT;//this is not pid.i************
	float response = pid->kp*err + pid->ki*pid->accumulate + pid->kd * (err - pid->last_err) / DT;
	pid->last_err = err;
	return response;
}
/*******************************************************************************
* heading_pose_botgui()
*******************************************************************************/
float heading_pose_botgui(pose_xyt_t target_pose, world_odometry bot_pose){
        float theta_temp = atan2(target_pose.y - bot_pose.y, target_pose.x - bot_pose.x) - bot_pose.theta;
        return theta_regulate(theta_temp);
}   

/*******************************************************************************
* void imu_get()
* 
*******************************************************************************/
void imu_get(int freq_div){
	float sample_imu_Hz = SAMPLE_RATE_HZ/freq_div;

	bot_data.accel_x = imu_data.accel[0];
	bot_data.accel_y = imu_data.accel[1];
	bot_data.accel_z = imu_data.accel[2];

	bot_data.roll_speed = (imu_data.dmp_TaitBryan[TB_ROLL_Y] - bot_data.roll)/sample_imu_Hz;
	bot_data.roll = imu_data.dmp_TaitBryan[TB_ROLL_Y];//*RAD_TO_DEG;
	bot_data.pitch_speed = (imu_data.dmp_TaitBryan[TB_PITCH_X] - bot_data.pitch)/sample_imu_Hz;
	bot_data.pitch = imu_data.dmp_TaitBryan[TB_PITCH_X];//*RAD_TO_DEG;
	bot_data.yaw_speed = (imu_data.dmp_TaitBryan[TB_YAW_Z] - bot_data.yaw)/sample_imu_Hz;
	bot_data.yaw = imu_data.dmp_TaitBryan[TB_YAW_Z];//*RAD_TO_DEG;

	bot_data.gyro_x = imu_data.gyro[0];
	bot_data.gyro_y = imu_data.gyro[1];
	bot_data.gyro_z = imu_data.gyro[2];

	//bot_data.imu_temp = imu_data.temp;

/*		imu_data.dmp_quat[QUAT_W], \
														imu_data.dmp_quat[QUAT_X], \
														imu_data.dmp_quat[QUAT_Y], \
														imu_data.dmp_quat[QUAT_Z]*/
	return;
}


/*******************************************************************************
* void barometer_get()
*  
*******************************************************************************/
void barometer_get(){
	if(rc_read_barometer()<0){
		fprintf(stderr,"\rERROR: Can't read Barometer");
		return;
	}
	// if we got here, new data was read and is ready to be accessed.
	// these are very fast function calls and don't actually use i2c
	bot_data.temp = rc_bmp_get_temperature();
	bot_data.pressure = rc_bmp_get_pressure_pa();
	bot_data.altitude = rc_bmp_get_altitude_m();
	bot_data.filtered_alt = rc_march_filter(&lowpass_alti,bot_data.altitude);
}


/*******************************************************************************
* GPS_loop() 
*
* GPS
*******************************************************************************/
void* GPS_loop(void* ptr){
	printf("GPS thread starts.\n");
	while(rc_get_state()!=EXITING){

		memset(gps_buf, 0, GPS_BUF_SIZE);
		//ret = rc_uart_read_bytes(GPS_BUS, GPS_bytes, &gps_buf[0]);
		ret = rc_uart_read_line(GPS_BUS, 128, &gps_buf[0]);
		if(ret<0) printf("Error reading bus\n");
		else if(ret==0)printf("timeout reached, %d bytes read\n", ret);
		else{
			 //bot_data.GPS_N = gps_buf;
			if(gps_buf[0] == '$' && gps_buf[1] == 'G' && gps_buf[2] == 'P' && gps_buf[3] == 'G' && gps_buf[4] == 'G' && gps_buf[5] == 'A'){
				//printf("%s\n", gps_buf);
				char *token;
				int i = 0;
				float GPS_N_temp, GPS_W_temp;
				token = strtok(gps_buf, ",");
				while( token != NULL ) 
				{
					//printf( " %s\n", token );
					token = strtok(NULL, ",");
					if(i == 0){
						//printf( " %f\n", atof(token) );
						bot_data.GPS_time_temp = atof(token);
					}
					if(i == 1){
						//printf( " %f\n", atof(token) );
						GPS_N_temp = atof(token);
					}
					if(i == 3){
						//printf( " %f\n", token );
						GPS_W_temp = atof(token);
					}
					i++;
				}
				//printf( "\nGPS GPGGA data numbers: %d\n", i );
				if(i > 7){
					bot_data.GPS_N = GPS_N_temp;
					bot_data.GPS_W = GPS_W_temp;
				}
							
			}

			
		}
		//usleep(1000000 / PRINTF_HZ);
	}
	printf("GPS thread ends.");
	return NULL;
}



/*******************************************************************************
* void on_pause_released() 
*	
* Make the Pause button toggle between paused and running states.
*******************************************************************************/
/*void on_pause_released(){
	rc_set_led(RED, OFF);
	printf("Pause Released\n");
	return;
}*/

/*******************************************************************************
* void on_pause_pressed() 
*
* If the user holds the pause button for 2 seconds, set state to exiting which 
* triggers the rest of the program to exit cleanly.
*******************************************************************************/
void on_pause_pressed(){
	if(record_flag == 0){
		record_flag = 1;
		rc_set_led(RED, ON);
	}
	else{
		record_flag = 0;
		rc_set_led(RED, OFF);
	}
	
	return;
}

/*******************************************************************************
* lcm_publish_loop() 

*******************************************************************************/
void* lcm_publish_loop(void* ptr){
	printf("LCM publish thread starts.\n");
	while(rc_get_state()!=EXITING){
		//publish lcm messages here
		pose_xyt_t_publish(lcm, ODOMETRY_CHANNEL, &pose_xyz);
		usleep(1000000 / PRINTF_HZ);
	}
	printf("LCM publish thread ends.\n");
	return NULL;
}