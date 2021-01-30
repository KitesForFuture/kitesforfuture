#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
    
#include "i2c_devices/interchip.c"
#include "i2c_devices/cat24c256.c"
#include "i2c_devices/bmp280.c"

struct i2c_bus bus0 = {14, 25};
struct i2c_bus bus1 = {18, 19};


void app_main(void)
{
    init_cat24(bus1);
    init_bmp28(bus1, readEEPROM(6));

    float test;

    printf("EEProm: ");
    test = readEEPROM(0);
    printf("%f\n", test);

    printf("BMP280 Pressure Diff: ");
    test = getPressureDiff();
    printf("%f\n", test);

    while(1) {
        vTaskDelay(5);
        printf(".");
    }
}