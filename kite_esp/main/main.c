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
#include "pid.c"

int counter = 0;

void init(){
	initUptime();
	initSensors();
	setRole(KITE);
	network_setup();
	initMotors();
	
	setAngle(1, 0);  //y+
	setSpeed(0, 0);  //y+
	setSpeed(3, 0);  //y-
	setAngle(2, 0);  //y-
	
	initMPU6050();
	initHeightSensorFusion();
	setNumberOfOmittedSends(0); // debugging info sent to pc every x iterations
	
}

void update(){
	
	if(counter > 20){
		counter = 0;
		printf("\nrot= %f\t%f\t%f\t\n     %f\t%f\t%f\t\n     %f\t%f\t%f\n", rot[0], rot[1], rot[2], rot[3], rot[4], rot[5], rot[6], rot[7], rot[8]);
		printf("accel_x = %f.\n", mpu_pos.accel_x-accel_offset_x);
		printf("accel_y = %f.\n", mpu_pos.accel_y-accel_offset_y);
		printf("accel_z = %f.\n", mpu_pos.accel_z-accel_offset_z);
		printf("wind: (%f, %f)\n", windY, windZ);
		printf("lineLength: %f, height = %f, kiteSpeed = %f\n", lineLength, getHeight(), kiteSpeed);
	}counter++;	
	
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
		setZAxisTrim(1.0*((float)receivedSignal[1])/(3003.0));
		setAUXTrim(1.0*((float)receivedSignal[2])/(3003.0));
		
		// START LANDING AT 35 seconds
		
		if(landing == false){
			setGoalHeight(15.0*((float)receivedSignal[5])/(3003.0));
			setYAxisTrim(2.5*((float)receivedSignal[0])/(3003.0));
			
			//if(queryTimer(t) > 35){
			
			if(timeSinceLastReceiveInSeconds() > 3.0 || (getBatteryPercentage() < 0.25 && getUptime() > 10)){
				setGoalHeight(-5);
				setYAxisTrim(0.0);
				landing = true;
			}
			
		}
		
		calculatePID();
	    
	    vTaskDelay(1.0);
	    
    }
    
}
