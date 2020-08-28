/* esp now receiver example
*/

#include <string.h>
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"

#include "../../COMMON_FILES/RC.c"

void app_main(void)
{
	printf("esp now RECEIVER here\n");
	
	setRole(DATA_RECEIVER);
	network_setup();
	
	
	while(1){
		
		
		
	    vTaskDelay(10.0);
	    
    }
}
