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
#include "driver/rtc_io.h"
#include "driver/i2c.h" // not needed in ground station ?
#include "driver/dac.h"

#include "mymath.c"

#include "my_gpio.c"
#include "dac.c"
#include "sensors.c"
#include "motors.c"
#include "timer.c"

#include "reel_in.c"

#include "../../COMMON_FILES/RC.c"


float speed = 0;
float potiValue = 0;
float potiValue2 = 0;
float offset, offset2 = 0;

#define REEL_IN_MODE 0
#define ENERGY_GENERATION_MODE 1

int GROUND_STATION_MODE = REEL_IN_MODE;

void init(){
	initMotors();
	initSensors();
	
	//networking
	setRole(KITE);
	network_setup();
	
	initDAC();
	initGPIO();
	initUptime();
	setNumberOfOmittedSends(0); // debugging info sent to pc every x iterations
	
	offset = 89*(((float)getSensor(3))/(3003.0));
	offset2 = 89*(((float)getSensor(1))/(3003.0));
	
	setSpeed(0, 0);
	setSpeed(1, 0);
	setSpeed(2, 0);
	setSpeed(3, 0);
}

float getVin(){
	return 11.0*(((float)getSensor(4) - 142)/(3003.0));
}

void setCurrent(float zeroToFive){
	setDACVoltage(25, zeroToFive*2.5/6.25);
}

Timer last_switching_event = 0;

void switchToReelInMode(){
	GROUND_STATION_MODE = REEL_IN_MODE;
	last_switching_event = startTimer();
	setGPIO_0(1);
	setGPIO_1(1);
	setGPIO_2(1);
}

void switchToEnergyGenerationMode(){
	GROUND_STATION_MODE = ENERGY_GENERATION_MODE;
	last_switching_event = startTimer();
	setGPIO_0(0);
	setGPIO_1(0);
	setGPIO_2(0);
}



float reelInSpeed = 0;
float getReelInCurrentOffset(){//TODO
	return 1;
}

void app_main(void){
	init();
	
	
	float v = 0;
	
	while(1){
		
		v += 0.01;
		if(v > 3.3) v = 0;
		
		getReelInCurrentInAmps();
		
		
		if(getUptime() < 5){
			// for the first 5 seconds, just let the ESC initialize
			setGPIO_0(1);
			setGPIO_1(1);
			setGPIO_2(1);
			setSpeed(0, 0);
			setSpeed(1, 0);
			setSpeed(2, 0);
			setSpeed(3, 0);
		}else{
			// read potentiometers
			potiValue = -offset + 89*(((float)getSensor(3))/(3003.0));
			potiValue = -potiValue + 0.001;
			
			potiValue2 = -offset2 + 89*(((float)getSensor(1))/(3003.0));
			potiValue2 = -potiValue2 + 0.001;
			
			//send interesting Data via WIFI
			sendData(potiValue, potiValue2, getReelInCurrentInAmps(), getVin(), 3.12*((float)getSensor(2)-142)/(3003.0), 3.12*((float)getSensor(5)-142)/(3003.0), 0, 0, lowRangeInAmps, highRangeInAmps);
			
			// control current
			// TODO: increase current proportional to Vin
			setCurrent(10*potiValue2/89.0);
				
			if(GROUND_STATION_MODE == REEL_IN_MODE){
				if(queryTimer(last_switching_event) > 0.5 && decideSpeed() == 1){
					switchToEnergyGenerationMode();
				}
				// control reel-in speed
				setSpeed(3, getSpeed());
			}else{
				//wait for 3 seconds AND V_in to be low, before switching back
				if(queryTimer(last_switching_event) > 1 && getVin() < 0.3){
					switchToReelInMode();
				}
				
				
			}
		}
		
	    vTaskDelay(1.0);
	    
    }
    
}
