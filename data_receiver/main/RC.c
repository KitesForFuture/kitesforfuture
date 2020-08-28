#define RECEIVER 0
#define SENDER 1

#define RC 0
#define DATA 1

#define DATALENGTH 10

float receivedSignal[DATALENGTH];

// HERE YOU CAN DEFINE THE ROLE AS SENDER OR RECEIVER:
int ROLE = RECEIVER;

static uint8_t broadcast_mac[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
const uint8_t WIFI_CHANNEL = 0;

int firstTime = 1;

typedef struct __attribute__((packed)) esp_now_msg_t
{
	uint32_t mode;
	uint32_t control[6];
	float data[DATALENGTH];
	// Can put lots of things here
} esp_now_msg_t;


static void msg_send_cb(const uint8_t* mac, esp_now_send_status_t sendStatus)
{
	switch (sendStatus){
		case ESP_NOW_SEND_SUCCESS:
			printf("Send success\n");
			break;
		
		case ESP_NOW_SEND_FAIL:
			printf("Send Failure\n");
			break;
		
		default:
			break;
	}
}

static void msg_recv_cb(const uint8_t *mac_addr, const uint8_t *data1, int len)
{
	//printf("message received. len = %d, sizeof(esp_now_msg_t) = %d\n", len, sizeof(esp_now_msg_t));
	if (len == sizeof(esp_now_msg_t))
	{
		esp_now_msg_t msg;
		memcpy(&msg, data1, len);
		printf("mode = %d\n", msg.mode);
		if(msg.mode == DATA){
			printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", receivedSignal[0], receivedSignal[1], receivedSignal[2], receivedSignal[3], receivedSignal[4], receivedSignal[5], receivedSignal[6], receivedSignal[7], receivedSignal[8], receivedSignal[9]);
		
			for(int i = 0; i < DATALENGTH; i++){
				receivedSignal[i] = msg.data[i];
			}
		}
	}
}

static void network_setup(void)
{
	// Flash FS
	nvs_flash_init();
	
	// Puts ESP in STATION MODE
	esp_wifi_set_mode(WIFI_MODE_STA);
	
	// Wifi
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	esp_wifi_init(&cfg);
	esp_wifi_start();
	
	// ESP NOW
	esp_now_init();
	
	//if(ROLE == SENDER){
		// Add peer
		esp_now_peer_info_t peer_info;
		peer_info.channel = WIFI_CHANNEL;
		memcpy(peer_info.peer_addr, broadcast_mac, 6);
		peer_info.ifidx = ESP_IF_WIFI_STA;
		peer_info.encrypt = false;
		esp_now_add_peer(&peer_info);
		
		// Register Send Callback
		esp_now_register_send_cb(msg_send_cb);
	//}else{
		// RECEIVER CALLBACK
		esp_now_register_recv_cb(msg_recv_cb);
	//}
}


static void send_msg(esp_now_msg_t * msg)
{
	// Pack
	uint16_t packet_size = sizeof(esp_now_msg_t);
	uint8_t msg_data[packet_size]; // Byte array
	memcpy(&msg_data[0], msg, sizeof(esp_now_msg_t));
	
	// Send
	esp_now_send(broadcast_mac, msg_data, packet_size);
}

static void sendData(uint32_t poti[6]){
	esp_now_msg_t msg;
	for(int i = 0; i < 6; i++){
		msg.control[i] = poti[i];
	}
	// Pack
	uint16_t packet_size = sizeof(esp_now_msg_t);
	uint8_t msg_data[packet_size]; // Byte array
	memcpy(&msg_data[0], &msg, sizeof(esp_now_msg_t));
	
	// Send
	esp_now_send(broadcast_mac, msg_data, packet_size);
}

