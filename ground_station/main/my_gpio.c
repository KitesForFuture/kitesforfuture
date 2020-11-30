#define GPIO_OUTPUT_IO_0    16
#define GPIO_OUTPUT_IO_1    17
#define GPIO_OUTPUT_IO_2    21
#define GPIO_OUTPUT_IO_3    22

#define GPIO_OUTPUT_PIN_SELECTION  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1) | (1ULL<<GPIO_OUTPUT_IO_2) | (1ULL<<GPIO_OUTPUT_IO_3))
void initGPIO()
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SELECTION;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

// set to high(1) or low(0)
void setGPIO_0(int level){
	gpio_set_level(GPIO_OUTPUT_IO_0, level);
}
void setGPIO_1(int level){
	gpio_set_level(GPIO_OUTPUT_IO_1, level);
}
void setGPIO_2(int level){
	gpio_set_level(GPIO_OUTPUT_IO_2, level);
}
void setGPIO_3(int level){
	gpio_set_level(GPIO_OUTPUT_IO_3, level);
}
