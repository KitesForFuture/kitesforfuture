/* servo motor control example
*/

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"

#include "driver/mcpwm.h"
#include "esp_adc_cal.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "mymath.c"

#include "timer.c"
#include "constants.c"
#include "sensors.c"
#include "motors.c"

#include "interchip.c"
#include "mpu6050.c"
#include "bmp280.c"
#include "heightSensorFusion.c"
#include "../../COMMON_FILES/RC2.c"
#include "windDirection.c"
#include "lineLength.c"
#include "sidewaysAngle.c"
#include "kiteSpeed.c"
#include "batteryPercentage.c"

#include "motor_safety_bounds.c"
#include "orientation_helpers.c"
#include "control_glide_elevator.c"
#include "control_glide_rudder.c"
#include "control_hover_elevator.c"
#include "control_hover_rudder.c"
#include "control_hover_height.c"

#include "pid.c"

//int counter = 0;

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
	
	initMPU6050();
	initHeightSensorFusion();
	setNumberOfOmittedSends(0); // debugging info sent to pc every x iterations
	
}

void update(){
	readMPURawData();
	processMPURawData();
	fuseHeightSensorData();
	updateBatteryPercentage();
	updateWindDirection();
	updateLineLength();
	updateSidewaysAngle();
	updateKiteSpeed();
}

void app_main(void){
	init();
	
	setGoalHeight(10);
	setRateOfClimb(0.5);
	
	int landing = false;
	
	
	while(1){
		
		update();
		
		// trim for balancing is between -0.3 and 0.3 (measured in radians with assumption sin(x)=x near 0)
		
		// set control gains from input signal
		
		LinksRechtsOffset = 140*((float)receivedSignal[0])/(3003.0); // [-70;70]
		setRateOfClimb(pow(10.,2.*((float)receivedSignal[1])/(3003.0)));
		
		//signal2 = ((float)receivedSignal[2])/(3003.0);
		//signal3 = ((float)receivedSignal[3])/(3003.0);
		
		//Pz = pow(10., 2.*((float)receivedSignal[0])/(3003.0));
		//Dz = pow(10., 2.*((float)receivedSignal[1])/(3003.0));
		
		// = 120.*((float)receivedSignal[2])/(3003.0);
		//Dz = pow(10., 2.*((float)receivedSignal[3])/(3003.0));
		HochRunterOffset = 100*((float)receivedSignal[4])/(3003.0); // [-50;50]


		
		if((float)receivedSignal[5] > -100){ // third knob from the right turns on manual kite fly mode, which ignores orientation
			flightMode = GLIDE_MODE;
		}else{
			flightMode = HOVER_MODE;
		}
		
		// START HOVER LAND AT 10% battery or signal loss
		
		if(landing == false){
			if((float)receivedSignal[5] > -750){
				setGoalHeight(-1);
			}else if((float)receivedSignal[5] > -2250){
				setGoalHeight(0);
			}else{
				setGoalHeight(1);
			}
			//setGoalHeight(-5.0 - 100.0*((float)receivedSignal[5])/(3003.0));
			setYAxisTrim(0.);
			
			//if(queryTimer(t) > 35){
			
			if(timeSinceLastReceiveInSeconds() > 3.0 || (getBatteryPercentage() < 0.10 && getUptime() > 10)){
				setGoalHeight(-5);
				setYAxisTrim(0.0);
				landing = true;
			}
			
		}
		
		calculatePID();
	    
	    
	    
	    
	    /*
	    if(counter == 10){
	    	//printf("angle = %f.\n", angle);
	    	
			printf("accel_x = %f, %f.\n", mpu_pos_avg.accel_x, mpu_pos.accel_x);
			printf("accel_y = %f, %f.\n", mpu_pos_avg.accel_y, mpu_pos.accel_y);
			printf("accel_z = %f, %f.\n", mpu_pos_avg.accel_z, mpu_pos.accel_z);
			printf("gyro_x = %f.\n", mpu_pos.gyro_x-mpu_pos_avg.gyro_x);
			printf("gyro_y = %f.\n", mpu_pos.gyro_y-mpu_pos_avg.gyro_y);
			printf("gyro_z = %f.\n", mpu_pos.gyro_z-mpu_pos_avg.gyro_z);
			printf("rot= %f\t%f\t%f\t\n     %f\t%f\t%f\t\n     %f\t%f\t%f\t", rot[0], rot[1], rot[2], rot[3], rot[4], rot[5], rot[6], rot[7], rot[8]);
			
			printf("free memory = %d.\n", xPortGetFreeHeapSize());
	    	counter = 0;
	    }counter++;
	    */
	    vTaskDelay(1.0);
	    
    }
    
}
