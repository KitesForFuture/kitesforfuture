#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt.h"
#include "freertos/queue.h" //TODO: is it used?

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"


#include "driver/mcpwm.h"
#include "esp_adc_cal.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

// ToDoLeo Up here for now cause other .c includes use it.
#define true 1
#define false 0

// some helper functions for mathematical operations
#include "kitemath/helpers.h"

#include "timer.c"
#include "constants.c"

// read data from analog sensors, e.g. battery state
#include "analog_sensors.c"
// servo motors and propeller motor control
#include "motors.c"
// i2c protocol for communication with mpu6050 and bmp280
#include "interchip.c"
// persistent memory chip
#include "cat24c256.c"
// gyroscope and accelerometer
#include "mpu6050.c"
// barometer
#include "bmp280.c"
// estimate current height from pressure and acceleration
#include "heightSensorFusion.c"

#include "../../COMMON_FILES/RC.c"

// determine wind direction
#include "windDirection.c"
// determine distance from ground station
#include "lineLength.c"
// angle towards wind direction in the plane parallel to ground
#include "sidewaysAngle.c"
// determine the kite's speed
#include "kiteSpeed.c"
// read battery state of charge from analog sensor
#include "batteryPercentage.c"

// rudder, elevator and motor speed control
#include "motor_safety_bounds.c"
#include "orientation_helpers.c"

#include "control_glide_elevator.c"
#include "control_glide_rudder.c"
#include "control_hover_elevator.c"
#include "control_hover_rudder.c"
#include "control_hover_height.c"
#include "PWM_input.c"
#include "pid.c"

//int counter = 0;

float time_difference; // ToDoLeo: This should be local in update as soon as we have everything refactored to get it passed in.
int64_t lastUpdateTime;
int64_t currentTime;

// rotation of the drone in world coordinates
float rot[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};


void init(){
	initUptime();
	initSensors();
	setRole(KITE);
	network_setup();
	initMotors();
	
	setAngle(TOP_RIGHT, 0);
	setSpeed(BOTTOM_LEFT, 0);
	setSpeed(BOTTOM_RIGHT, 0);
	setAngle(TOP_LEFT, 0);
	
	initMPU6050(readEEPROM(0), readEEPROM(1), readEEPROM(2), readEEPROM(3), readEEPROM(4), readEEPROM(5));
	initHeightSensorFusion();
	setNumberOfOmittedSends(0); // debugging info sent to pc every x iterations
	
	initPWMInput();
}

void update(){

	// int64_t esp_timer_get_time() returns time since boot in us (microseconds = 10^-6 seconds)	
	lastUpdateTime = currentTime;
	currentTime = esp_timer_get_time();
	time_difference = 0.000001*(float)(currentTime - lastUpdateTime);

	readMPURawData();
	processMPURawData(time_difference, rot);
	fuseHeightSensorData();
	updateBatteryPercentage();
	updateWindDirection();
	updateLineLength();
	updateSidewaysAngle();
	updateKiteSpeed();
	updatePWMInput();
}

void app_main(void){
	init();
	
	setGoalHeight(10);
	setRateOfClimb(0.5);
	
	int landing = false;
	
	// Do this immediately before the first update.
	currentTime = esp_timer_get_time();
	vTaskDelay(1.0);
	while(1){
		
		update();
		
	    smoothedSWC = 0.9*smoothedSWC + 0.1*getPWMInput0to1normalized(0);
	    
		// set control gains from input signal
		
		LinksRechtsOffset = 140*((float)receivedSignal[0])/(3003.0); // [-70;70]
		HochRunterOffset = 100*((float)receivedSignal[4])/(3003.0); // [-50;50]
		
		//setRateOfClimb(pow(10.,2.*((float)receivedSignal[1])/(3003.0)));
		
		//if((float)receivedSignal[5] > -100){ // third knob from the right turns on manual kite fly mode, which ignores orientation
		if(smoothedSWC < 0.25){
			gotoGlideMode();
		}else{
			gotoHoverMode();
		}
		
		// START HOVER LAND AT 10% battery or signal loss
		
		if(landing == false){
			/*
			if((float)receivedSignal[5] > -750){
				setGoalHeight(-1); // go down
			}else if((float)receivedSignal[5] > -2250){
				setGoalHeight(0); // keep height
			}else{
				setGoalHeight(1); // go up
			}
			*/
			if(smoothedSWC > 0.75){
				setGoalHeight(-1); // go down
			}else{
				setGoalHeight(1); // go up
			}
			//setGoalHeight(-5.0 - 100.0*((float)receivedSignal[5])/(3003.0));
			setYAxisTrim(0.); // ???
			
			//if(queryTimer(t) > 35){
			
			if(/*timeSinceLastReceiveInSeconds() > 3.0 ||*/ (getBatteryPercentage() < 0.10 && getUptime() > 10)){
				setGoalHeight(-1);
				setYAxisTrim(0.0); // ???
				landing = true;
			}
			
		}
		
		calculatePID();
		
		
	    
	    // OVERWRITE CONTROLS BY MANUAL RC
	    if(smoothedSWC < 0.25){
	    	servoElevator = getPWMInputMinus1to1normalized(2)*60;
	    	servoRudder = getPWMInputMinus1to1normalized(1)*60;
	    	
	    	motorLeft = getPWMInput0to1normalized(3)*90;
	    	motorRight = getPWMInput0to1normalized(3)*90;
		}
		
		setAngle(TOP_RIGHT, servoElevator);
		setAngle(TOP_LEFT, -servoRudder);
		//if(getUptime() > 5){
			setSpeed(BOTTOM_LEFT, motorLeft);
			setSpeed(BOTTOM_RIGHT, motorRight);
		//}
	    
	    
	    
	
		// SENDING DEBUGGING DATA TO GROUND
		sendData(smoothedSWC, getPWMInputMinus1to1normalized(1), getPWMInputMinus1to1normalized(2), rot0, rot3, rot6, rot1, rot4, rot7, rot2, rot5, rot8, getHeight(), mpu_pos.accel_x, mpu_pos.accel_y, mpu_pos.accel_z, mpu_pos.gyro_x, mpu_pos.gyro_y, mpu_pos.gyro_z, motorLeft, servoRudder, servoElevator, getBatteryPercentage());
	    

	    vTaskDelay(1.0);
	    
    }
    
}
