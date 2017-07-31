/*******************************************************************************
*
*	imu_lcm_example.c
*   Publishes IMU data to "BALANCEBOT_IMU_DATA" as an example of how to publish 
*   to LCM 
*
*	pgaskell@umich.edu
*******************************************************************************/

#include <lcm/lcm.h>
// usefulincludes is a collection of common system includes for the lazy
// This is not necessary for roboticscape projects but here for convenience
#include <rc_usefulincludes.h> 
// main roboticscape API header
#include <roboticscape.h>
#include "../lcmtypes/balancebot_imu_t.h"

#define 	SAMPLE_RATE_HZ 			100	  // imu read speed
#define 	DT 						0.01  // 1/sample_rate
#define		LCM_HZ				    20

// Global Variables
rc_imu_data_t imu_data;
balancebot_imu_t balancebot_imu_data;

// LCM Global Variables
lcm_t * lcm = NULL;
static const char IMU_DATA[] = "BALANCEBOT_IMU_DATA";

// IMU interrupt routine
void read_imu();

//threads
void* lcm_publish_loop(void* ptr);


int main(int argc, char *argv[]){
	//initialize robocape
	rc_initialize();
	rc_set_cpu_freq(FREQ_1000MHZ);
    lcm = lcm_create(NULL);

    printf("Starting LCM Thread\n");
	//start lcm publish thread
	pthread_t  lcm_publish_thread;
	pthread_create(&lcm_publish_thread, NULL, lcm_publish_loop, (void*) NULL);

	// set up IMU configuration
	rc_imu_config_t imu_config = rc_default_imu_config();
	imu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	imu_config.orientation = ORIENTATION_Z_UP;

    printf("Starting IMU Thread\n");
	// now set up the imu for dmp interrupt operation
	if(rc_initialize_imu_dmp(&imu_data, imu_config)){
		printf("rc_initialize_imu_failed\n");
		return -1;
	}

	//attach control routine to imu interrupt
	rc_set_imu_interrupt_func(&read_imu);
	
	// start in the RUNNING state
	rc_set_state(RUNNING);

	// Keep Running until state changes to EXITING
	while(rc_get_state()!=EXITING){
		// always sleep at some point
		usleep(10000);
	}
	
	rc_cleanup(); // exit cleanly
	return 0;
}


/*******************************************************************************
* read_imu() IMU interrupt routine to populate lcm message
* Called at SAMPLE_RATE_HZ
*******************************************************************************/
void read_imu(){

	/******************************************************************
	* STATE_ESTIMATION
	* read IMU and fill lcm message
	******************************************************************/

    balancebot_imu_data.accel[0] = imu_data.accel[0];
    balancebot_imu_data.accel[1] = imu_data.accel[1];
    balancebot_imu_data.accel[2] = imu_data.accel[2];


	/*************************************************************
	* check for various exit conditions AFTER state estimate
	***************************************************************/
	if(rc_get_state() == EXITING){
		return;
	}
	return;
}

/*******************************************************************************
* lcm publish_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*******************************************************************************/

void* lcm_publish_loop(void* ptr){
	while(rc_get_state()!=EXITING){
		//publish lcm messages here, always publishes the latest data
        balancebot_imu_t_publish(lcm, IMU_DATA, &balancebot_imu_data);
		usleep(1000000 / LCM_HZ);
	}
	return NULL;
}

