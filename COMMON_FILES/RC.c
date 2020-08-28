#define KITE 0
#define DATA_RECEIVER 1
#define RADIO_CONTROL 2

#define RC_MODE 0
#define DATA_MODE 1

#define DATALENGTH 10

// HERE YOU CAN DEFINE THE ROLE AS SENDER OR RECEIVER:
int ROLE = KITE;

static uint8_t broadcast_mac[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
const uint8_t WIFI_CHANNEL = 0;

int firstTime = 1;

int signalOffset[6] = {0,0,0,0,0,0};
int receivedSignal[6] = {0,0,0,0,0,0};
float receivedData[DATALENGTH] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

typedef struct __attribute__((packed)) esp_now_msg_t
{
	uint32_t mode;
	uint32_t control[6];
	float data[DATALENGTH];
	// Can put lots of things here
} esp_now_msg_t;


void setRole(int role){
	ROLE = role;
}

static void msg_send_cb(const uint8_t* mac, esp_now_send_status_t sendStatus)
{
	//printf("send_cb called");
	switch (sendStatus){
		case ESP_NOW_SEND_SUCCESS:
			//printf("Send success\n");
			break;
		
		case ESP_NOW_SEND_FAIL:
			//printf("Send Failure\n");
			break;
		
		default:
			break;
	}
}

// time since boot in us (microseconds = 10^-6 seconds) when msg_recv_cb was last called.
int64_t lastReceivedtime = 0;

float timeSinceLastReceiveInSeconds(){
	int64_t currentTime = esp_timer_get_time();
	return 0.000001*(float)(currentTime - lastReceivedtime);
}

static void msg_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
	if(ROLE == KITE){
		//printf("message received. len = %d, sizeof(esp_now_msg_t) = %d\n", len, sizeof(esp_now_msg_t));
		if (len == sizeof(esp_now_msg_t))
		{
			esp_now_msg_t msg;
			memcpy(&msg, data, len);
			
			if(msg.mode == RC_MODE){
				
				// int64_t esp_timer_get_time() returns time since boot in us (microseconds = 10^-6 seconds)
				lastReceivedtime = esp_timer_get_time();
				
				// HERE YOU CAN DEFINE THE BEHAVIOUR ON RECEIVING DATA:
				
				// IF the sender tells you that it just turned on ...
				if(msg.control[0] >= 1000000){
					signalOffset[0] = (int)msg.control[0]-1000000;
					for(int i = 1; i < 6; i++){
						signalOffset[i] = (int)msg.control[i];
					}
					firstTime = 0;
				// if you just turned on yourself ...
				}else if(firstTime == 1){
					for(int i = 0; i < 6; i++){
						signalOffset[i] = (int)msg.control[i];
					}
					firstTime = 0;
				// in usual operation
				}else{
					for(int i = 0; i < 6; i++){
						receivedSignal[i] = (int)msg.control[i] - signalOffset[i];
					}
				}
			}
		}
	}else if (ROLE == DATA_RECEIVER){
		//printf("message received. len = %d, sizeof(esp_now_msg_t) = %d\n", len, sizeof(esp_now_msg_t));
		if (len == sizeof(esp_now_msg_t))
		{
			esp_now_msg_t msg;
			memcpy(&msg, data, len);
			if(msg.mode == DATA_MODE){
				
				for(int i = 0; i < DATALENGTH; i++){
					receivedData[i] = msg.data[i];
				}
				printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", receivedData[0], receivedData[1], receivedData[2], receivedData[3], receivedData[4], receivedData[5], receivedData[6], receivedData[7], receivedData[8], receivedData[9]);
			}
		}
	}
	
}
static void network_setup(void)
{
	
	// Initialize FS NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
	
	
	//ESP_ERROR_CHECK(esp_netif_init());
	
	// Wifi
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	esp_wifi_init(&cfg);
	
	
    //ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
	// Puts ESP in STATION MODE
	
	esp_wifi_set_mode(WIFI_MODE_STA); // MUST BE CALLED AFTER esp_wifi_init(&cfg) to have a non-volatile effect on the flash nvs!
	
	esp_wifi_start();
	
	#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    //	ESP_ERROR_CHECK( esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
	#endif
	
	// ESP NOW
	esp_now_init();
	
	if(ROLE == RADIO_CONTROL || ROLE == KITE){
		// Add peer
		esp_now_peer_info_t peer_info;
		peer_info.channel = WIFI_CHANNEL;
		memcpy(peer_info.peer_addr, broadcast_mac, 6);
		peer_info.ifidx = ESP_IF_WIFI_STA;
		peer_info.encrypt = false;
		esp_now_add_peer(&peer_info);
		
		// Register Send Callback
		esp_now_register_send_cb(msg_send_cb);
	}
	
	if(ROLE == KITE || ROLE == DATA_RECEIVER){
		// RECEIVER CALLBACK
		esp_now_register_recv_cb(msg_recv_cb);
	}
}

static void sendControl(uint32_t poti[6]){
	esp_now_msg_t msg;
	msg.mode = RC_MODE;
	if(firstTime == 1){
		msg.control[0] = poti[0] + 1000000;
		firstTime = 0;
	}else{
		msg.control[0] = poti[0];
	}
	for(int i = 1; i < 6; i++){
		msg.control[i] = poti[i];
	}
	for(int i = 0; i < DATALENGTH; i++){
		msg.data[i] = 0.0;
	}
	
	// Pack
	uint16_t packet_size = sizeof(esp_now_msg_t);
	uint8_t msg_data[packet_size]; // Byte array
	memcpy(&msg_data[0], &msg, sizeof(esp_now_msg_t));
	
	// Send
	esp_now_send(broadcast_mac, msg_data, packet_size);
}

static void sendData(float data[DATALENGTH]){

	esp_now_msg_t msg;
	
	msg.mode = DATA_MODE;
	for(int i = 0; i < 6; i++){
		msg.control[i] = 0;
	}
	for(int i = 0; i < DATALENGTH; i++){
		msg.data[i] = data[i];
	}
	
	// Pack
	uint16_t packet_size = sizeof(esp_now_msg_t);
	uint8_t msg_data[packet_size]; // Byte array
	memcpy(&msg_data[0], &msg, sizeof(esp_now_msg_t));
	
	// Send
	esp_now_send(broadcast_mac, msg_data, packet_size);
}


