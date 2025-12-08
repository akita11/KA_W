// KA_W Server
// Configuration
// PC--[USB]--KAW[S] <---[ESPNow]--> KAW[C]

#define NUM_CLIENTS 2

#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Ticker.h>
// FastLED for AtomS3 Lite onboard LED
#include <FastLED.h>
#include <esp_now.h>

// Received data queue and task
typedef struct {
	uint8_t mac[6];
	uint8_t data[256];
	int len;
} RecvItem;

static QueueHandle_t recvQueue = NULL;

// Forward declarations for variables used in recvTask
extern uint32_t lastReceiveTime;
extern uint32_t previousThroughputSamplingTime;
extern uint16_t nThroughputSamples;
#define SERVER_THROUGHPUT_SAMPLE 100

volatile uint8_t beaconEnabled = 0; // STARTで1, STOPで0

volatile uint8_t fReceived = 0;
volatile uint32_t prevReceiveTime = 0;
volatile char prevClientID = 0;

// データ保持用配列: CLIENT x 12サンプル の yaw, roll, pitch
float client_yaw[NUM_CLIENTS][12];
float client_roll[NUM_CLIENTS][12];
float client_pitch[NUM_CLIENTS][12];
volatile uint32_t lastDisplayTime = 0;

// FastLED fallback
#define FALLBACK_NUM_LEDS 1
static CRGB fastled_leds[FALLBACK_NUM_LEDS];
static bool fastled_initialized = false;
static int fastled_pin = -1;

// Callback function for when data is received - enqueue and return quickly
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
	if (recvQueue == NULL) return;
	RecvItem item;
	memcpy(item.mac, mac, 6);
	int copyLen = len;
	if (copyLen > (int)sizeof(item.data)) copyLen = sizeof(item.data);
	memcpy(item.data, incomingData, copyLen);
	item.len = copyLen;

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	// try FromISR first (fast and safe if called in ISR-like context)
	BaseType_t ok = xQueueSendFromISR(recvQueue, &item, &xHigherPriorityTaskWoken);
	if (ok != pdTRUE) {
		// fallback to non-ISR enqueue (non-blocking)
		ok = xQueueSend(recvQueue, &item, 0);
	}
	if (xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}

uint8_t nShowDebug = 0;
uint32_t sampleSeq = 0;

// Task to process received items
void recvTask(void *pvParameters) {
	RecvItem item;
	for (;;) {
		if (xQueueReceive(recvQueue, &item, portMAX_DELAY) == pdTRUE) {
			uint32_t currentTime = millis();
/*
			nShowDebug++;
			if (nShowDebug < 2){
			// データ受信時に内容とmillis()を表示
				printf("[SERVER] Data received at %lu ms: %d bytes\n", (unsigned long)currentTime, item.len);
				for (int i = 0; i < item.len; i++) {
					char c = item.data[i];
					if (c >= 32 && c <= 126) putchar(c); else putchar('.');
				}
				putchar('\n');
			}
			nShowDebug = (nShowDebug + 1) % 24;
*/
			//printf("%d %c\n", currentTime - lastReceiveTime, item.data[0]);
			//printf("%s\n", item.data);
			char id = item.data[0];
			// 交互受信チェック
			if (prevClientID != 0 && ((prevClientID == '1' && id != '2') || (prevClientID == '2' && id != '1'))) {
				printf("Error: Expected CLIENT %c but received %c\n", (prevClientID == '1') ? '2' : '1', id);
				fastled_leds[0] = CRGB(30, 0, 0); // 赤
				FastLED.show();
			}
			prevClientID = id;
			// データパースと格納
			for (uint8_t i = 0; i < 12; i++){
				char sample[19];
				memcpy(sample, &item.data[1 + i * 18], 18);
				sample[18] = '\0';
				char yaw_str[7], roll_str[7], pitch_str[7];
				memcpy(yaw_str, &sample[0], 6); yaw_str[6] = '\0';
				memcpy(roll_str, &sample[6], 6); roll_str[6] = '\0';
				memcpy(pitch_str, &sample[12], 6); pitch_str[6] = '\0';
				int iyaw = atoi(yaw_str);
				int iroll = atoi(roll_str);
				int ipitch = atoi(pitch_str);
				float yaw = iyaw / 1000.0f;
				float roll = iroll / 1000.0f;
				float pitch = ipitch / 1000.0f;
				int client_idx = id - '1'; // '1' -> 0, '2' -> 1
				client_yaw[client_idx][i] = yaw;
				client_roll[client_idx][i] = roll;
				client_pitch[client_idx][i] = pitch;
			}
			// CLIENT=2受信後に全データ表示
			if (id == '2') {
				uint32_t now = millis();
				uint32_t elapsed = (lastDisplayTime == 0) ? 0 : (now - lastDisplayTime);
				//printf("Elapsed: %d ms\n", elapsed);
				for (int i = 0; i < 12; i++) {
//					printf("%d %.3f %.3f %.3f %.3f %.3f %.3f\n", sampleSeq++, client_yaw[0][i], client_roll[0][i], client_pitch[0][i], client_yaw[1][i], client_roll[1][i], client_pitch[1][i]);
				}
				lastDisplayTime = now;
			}				
			lastReceiveTime = currentTime;
			fReceived = (fReceived + 1) % 10;
			uint32_t interval = (prevReceiveTime == 0) ? 0 : (currentTime - prevReceiveTime);
			uint8_t clientID = item.data[0] - '0';
			//printf("Received %d bytes from %d Interval: %lu ms\n", item.len, clientID, (unsigned long)interval);
			if (interval > 40) { // 受信間隔が24msよりも大幅に長い場合
				if (fReceived < 5) {
					fastled_leds[0] = CRGB(30, 0, 30); // 赤紫
				} else {
					fastled_leds[0] = CRGB(30, 0, 0); // 赤
				}
			} else {
				if (fReceived < 5) {
					fastled_leds[0] = CRGB(0, 0, 30); // 青
				} else {
					fastled_leds[0] = CRGB(30, 30, 0); // 黄
				}
			}
			FastLED.show();
			// スループット計算
			prevReceiveTime = currentTime;
			nThroughputSamples++;
			if (nThroughputSamples == SERVER_THROUGHPUT_SAMPLE) {
				uint32_t timeSinceLast = (previousThroughputSamplingTime == 0) ? 0 : (currentTime - previousThroughputSamplingTime);
				float avgTime = (float)timeSinceLast / (float)SERVER_THROUGHPUT_SAMPLE;
				nThroughputSamples = 0;
				printf("Average interval: %.2f ms / %d - %d\n", avgTime, currentTime, previousThroughputSamplingTime);
				previousThroughputSamplingTime = currentTime;
			}
		}
	}
}

#define SAMPLE_FREQ 250
// TDMA parameters

#define TDMA_FRAME_MS 48 // Frame period in milliseconds (12サンプル x 4ms)
#define TDMA_BEACON0 0    // beacon payload first byte marker
#define TDMA_BEACON1 1    // beacon payload second byte marker
Ticker beaconTicker;

// Communication task and queue
typedef struct {
	uint8_t addr[6];
	uint8_t data[32];
	size_t len;
} SendItem;

static QueueHandle_t sendQueue = NULL;
static TaskHandle_t commTaskHandle = NULL;

void commTask(void *pvParameters) {
	SendItem item;
	vTaskDelay(pdMS_TO_TICKS(500)); // Wait for WiFi/ESP-NOW to fully initialize
	for(;;) {
		if (xQueueReceive(sendQueue, &item, portMAX_DELAY) == pdTRUE) {
			vTaskDelay(pdMS_TO_TICKS(2)); // Small delay before send
			esp_err_t res = esp_now_send(item.addr, item.data, item.len);
			if (res != ESP_OK) {
				printf("commTask: esp_now_send failed %d\n", res);
			}
		}
	}
}

// ESP-NOW configuration
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast address
char packetBuffer[256];																						 // Buffer to accumulate packets
int packetCount = 0;																							 // Current number of packets in buffer

//char buf[1025];

uint32_t t0;
uint32_t lastReceiveTime = 0; // Track last receive time for SERVER
// Throughput measurement (server)
#define SERVER_THROUGHPUT_SAMPLE 100
uint16_t nThroughputSamples = 0;
uint32_t previousThroughputSamplingTime = 0;

static void initFastLEDFallback()
{
	if (fastled_initialized)
		return;
	// Use ATOMS3 Lite LED pin 35
	fastled_pin = 35;
	FastLED.addLeds<WS2812B, 35, GRB>(fastled_leds, FALLBACK_NUM_LEDS);
	FastLED.setBrightness(64);
	fastled_initialized = true;
	fastled_leds[0] = CRGB::Green;
	FastLED.show();
}

// Callback function for when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
	if (status != ESP_NOW_SEND_SUCCESS)
	{
		printf("Last Packet Send Status = FAILED");
	}
//	else printf("Last Packet Send Status = SUCCESS");
}

uint32_t lastBeaconSentTime = 0;

void setup()
{
	M5.begin();

	// Initialize FastLED fallback unconditionally (do not use M5.Led)
	initFastLEDFallback();
	// ensure initial color is green (stopped)
	fastled_leds[0] = CRGB(0, 30, 0);
	FastLED.show();

	// Initialize ESP-NOW in STA mode (simpler, avoids AP overhead)
	WiFi.mode(WIFI_MODE_STA);
	WiFi.channel(1); // チャンネルを固定して通信安定化
	vTaskDelay(pdMS_TO_TICKS(100));
	if (esp_now_init() != ESP_OK)
	{
		printf("Error initializing ESP-NOW\n");
		return;
	}
	// Register receive callback
	esp_now_register_recv_cb(onDataRecv);
	// Register send callback so we can observe send results on server as well
	esp_now_register_send_cb(onDataSent);
	// Register broadcast peer explicitly before starting beacon
	if (!esp_now_is_peer_exist(broadcastAddress)) {
		esp_now_peer_info_t peer = {};
		memcpy(peer.peer_addr, broadcastAddress, 6);
		peer.channel = 0;
		peer.encrypt = false;
		esp_now_add_peer(&peer);
	}
	printf("ESP-NOW initialized in STA mode (receiving)\n");
	vTaskDelay(pdMS_TO_TICKS(100)); // Small delay after ESP-NOW init
	// create send queue and communication task (pinned to core 0)
	sendQueue = xQueueCreate(10, sizeof(SendItem));
	if (sendQueue == NULL) {
		printf("Failed to create sendQueue\n");
	} else {
		xTaskCreatePinnedToCore(commTask, "CommTask", 4096, NULL, configMAX_PRIORITIES-2, &commTaskHandle, 0);
	}
	// create receive queue and task
	recvQueue = xQueueCreate(16, sizeof(RecvItem));
	if (recvQueue == NULL) {
		printf("Failed to create recvQueue\n");
	} else {
		xTaskCreatePinnedToCore(recvTask, "RecvTask", 4096, NULL, configMAX_PRIORITIES-3, NULL, 0);
	}
	// setup()または初期化部で必ずbeaconTickerを起動
	beaconTicker.attach_ms(TDMA_FRAME_MS, [](){
		if (!beaconEnabled) return;
		if (sendQueue == NULL) return;
		SendItem item;
		memcpy(item.addr, broadcastAddress, 6);
		item.data[0] = 0xBE; item.data[1] = 0xAC; item.len = 2;
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendFromISR(sendQueue, &item, &xHigherPriorityTaskWoken);
		if (xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
		//printf("[SERVER] Beacon sent at %lu ms\n", millis());
//		printf("%d\n", millis() - lastBeaconSentTime);
//		lastBeaconSentTime = millis();
	});
}

void loop()
{
	// Check for button press to send commands
	M5.update();
	static uint8_t lastButtonState = 0;
	static uint32_t lastButtonTime = 0;

	if (M5.BtnA.wasClicked()) {
		beaconEnabled = !beaconEnabled;
		//printf("beaconEnabled = %d\n", beaconEnabled);
		if (beaconEnabled) {
			sampleSeq = 0;
			fastled_leds[0] = CRGB(30, 30, 0);
		} else {
			delay(100);
			fastled_leds[0] = CRGB(0, 30, 0);
		}
		FastLED.show();
	}
	// ESP-NOW server receives data via callback (onDataRecv)
	// No polling needed, data is received asynchronously
	delay(100);
}
