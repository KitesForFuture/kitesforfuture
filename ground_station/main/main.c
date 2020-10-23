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

#include "sensors.c"
#include "motors.c"


float speed = 0;
float offset = 0;

void init(){
	initMotors();
	initSensors();
	
	offset = 89*(((float)getSensor(3))/(3003.0));
	
	setSpeed(0, 0);
	setSpeed(1, 0);
	setSpeed(2, 0);
	setSpeed(3, 0);
}


void app_main(void){
	init();
	
	while(1){
		speed = -offset + 89*(((float)getSensor(3))/(3003.0));
		speed = -speed + 0.001;
		printf("speed = %f\n", speed);
		setSpeed(0, speed);
		setSpeed(1, speed);
		setSpeed(2, speed);
		setSpeed(3, speed);
		
	    vTaskDelay(1.0);
	    
    }
    
}
