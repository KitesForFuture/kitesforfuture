

/**
 * GPIO status:
 * GPIO2:  input, pulled up, interrupt from rising edge and falling edge
 */

#define GPIO_INPUT_IO_0     2
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_0)
#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle gpio_evt_queue = NULL;


float rise_to_rise_time = 1;
int64_t last_rise_time = 0;
float duty = 1;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            
            if(gpio_get_level(io_num) == 1){
            	// rising edge
            	int64_t currentTime = esp_timer_get_time();
				rise_to_rise_time = (float)(currentTime - last_rise_time);
				last_rise_time = currentTime;
            	
            }else if (gpio_get_level(io_num) == 0){
            	// falling edge
            	int64_t currentTime = esp_timer_get_time();
            	if(rise_to_rise_time != 0){
	            	duty = 20*( (float)(currentTime - last_rise_time) ) / rise_to_rise_time -1;
	            	if(duty < 0) duty = 0;
	            	if(duty > 1) duty = 1;
	            }
            }
            printf("GPIO[%d] intr, val: %d, duty = %f, rr_time = %f, lr_time = %lld\n", io_num, gpio_get_level(io_num), duty, rise_to_rise_time, last_rise_time);
            
            
            //setSpeed(BOTTOM_LEFT, motorLeft);
			//setSpeed(BOTTOM_RIGHT, motorRight);
        }
    }
}

void initPWM_Input()
{
    gpio_config_t io_conf;

    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
	
	//TODO: just to show that it's possible?
	
    //remove isr handler for gpio number.
    gpio_isr_handler_remove(GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin again
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);

	//TODO: this looks useful:
	//vTaskDelay(1000 / portTICK_RATE_MS);
}

