// KA_W Server
// Configuration
// PC--[USB]--KAW[S] <---[ESPNow]--> KAW[C]

#define NUM_CLIENTS 2

#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiServer.h>
#include <WiFiClient.h>
#include <Ticker.h>
// FastLED for AtomS3 Lite onboard LED
#include <FastLED.h>

// Received data queue and task
typedef struct {
	char clientID;
	uint8_t data[256];
	int len;
} RecvItem;

typedef struct {
	char id;
	float yaw[24];
	float roll[24];
	float pitch[24];
	uint32_t tm;
	uint8_t fAlternatingError;
} DisplayItem;

static QueueHandle_t recvQueue = NULL;
static QueueHandle_t displayQueue = NULL;

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

// TCP server
WiFiServer tcpServer(12345); // Port 12345
WiFiClient clients[2]; // Assuming 2 clients

uint8_t nShowDebug = 0;
uint32_t sampleSeq = 0;

uint32_t t0 = 0;

uint8_t fAlternatingReceiveError = 0;

// Task to process received items
void recvTask(void *pvParameters) {
	RecvItem item;
	for (;;) {
		// Check for new clients
		WiFiClient newClient = tcpServer.available();
		if (newClient) {
			for (int i = 0; i < NUM_CLIENTS; i++) {
				if (!clients[i]) {
					clients[i] = newClient;
					printf("Client %d connected\n", i+1);
					break;
				}
			}
		}

		// Read data from clients
		for (int i = 0; i < NUM_CLIENTS; i++) {
			if (clients[i] && clients[i].available()) {
				int len = clients[i].readBytes((char*)item.data, sizeof(item.data));
				if (len > 0) {
					item.clientID = '1' + i; // '1' for i=0, '2' for i=1
					item.len = len;
					processReceivedData(item);
				}
			}
		}
		vTaskDelay(pdMS_TO_TICKS(10)); // Small delay
	}
}

void processReceivedData(RecvItem item) {
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
				fAlternatingReceiveError = 1;
				//printf("Error: Expected CLIENT %c but received %c\n", (prevClientID == '1') ? '2' : '1', id);
				//fastled_leds[0] = CRGB(30, 0, 0); // 赤
				//FastLED.show();
			}
			else fAlternatingReceiveError = 0;
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
			if (id == '2' && beaconEnabled) {
				uint32_t now = millis();
				uint32_t elapsed = (lastDisplayTime == 0) ? 0 : (now - lastDisplayTime);
				//printf("Elapsed: %d ms\n", elapsed);
				uint32_t t1 = micros();
				uint32_t tm = t1 - t0;
				t0 = t1;
				char cAlternatingError = fAlternatingReceiveError ? '*' : ' ';
				DisplayItem dispItem;
				dispItem.id = id;
				dispItem.tm = tm;
				dispItem.fAlternatingError = fAlternatingReceiveError;
				for (int c = 0; c < NUM_CLIENTS; c++) {
					for (int i = 0; i < 12; i++) {
						dispItem.yaw[c*12 + i] = client_yaw[c][i];
						dispItem.roll[c*12 + i] = client_roll[c][i];
						dispItem.pitch[c*12 + i] = client_pitch[c][i];
					}
				}
				xQueueSend(displayQueue, &dispItem, 0);
				lastDisplayTime = now;
			}				
			lastReceiveTime = currentTime;
			fReceived = (fReceived + 1) % 10;
			uint32_t interval = (prevReceiveTime == 0) ? 0 : (currentTime - prevReceiveTime);
			uint8_t clientID = item.data[0] - '0';
			//printf("Received %d bytes from %d Interval: %lu ms\n", item.len, clientID, (unsigned long)interval);
			// 受信間隔エラーをLED表示
/*
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
*/
			// スループット計算
/*
			prevReceiveTime = currentTime;
			nThroughputSamples++;
			if (nThroughputSamples == SERVER_THROUGHPUT_SAMPLE) {
				uint32_t timeSinceLast = (previousThroughputSamplingTime == 0) ? 0 : (currentTime - previousThroughputSamplingTime);
				float avgTime = (float)timeSinceLast / (float)SERVER_THROUGHPUT_SAMPLE;
				nThroughputSamples = 0;
				//printf("Average interval: %.2f ms / %d - %d\n", avgTime, currentTime, previousThroughputSamplingTime);
				previousThroughputSamplingTime = currentTime;
			}
*/
		}
	}
}

#define SAMPLE_FREQ 250

// TDMA parameters
#define TDMA_FRAME_MS 48	// Frame period in milliseconds (12サンプル x 4ms)
#define TDMA_BEACON0 0    // beacon payload first byte marker
#define TDMA_BEACON1 1    // beacon payload second byte marker
Ticker beaconTicker;

// Communication task and queue
typedef struct {
	uint8_t data[32];
	size_t len;
} SendItem;

static QueueHandle_t sendQueue = NULL;
static TaskHandle_t commTaskHandle = NULL;

void commTask(void *pvParameters) {
	SendItem item;
	vTaskDelay(pdMS_TO_TICKS(500)); // Wait for WiFi/TCP to fully initialize
	for(;;) {
		if (xQueueReceive(sendQueue, &item, portMAX_DELAY) == pdTRUE) {
			vTaskDelay(pdMS_TO_TICKS(2)); // Small delay before send
			// Send to all connected clients
			for (int i = 0; i < NUM_CLIENTS; i++) {
				if (clients[i] && clients[i].connected()) {
					clients[i].write(item.data, item.len);
				}
			}
		}
	}
}

// ESP-NOW configuration
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast address
char packetBuffer[256];																						 // Buffer to accumulate packets
int packetCount = 0;																							 // Current number of packets in buffer

//char buf[1025];

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
	Serial.setTxBufferSize(2048); // Increase TX buffer to reduce blocking

	// Initialize FastLED fallback unconditionally (do not use M5.Led)
	initFastLEDFallback();
	// ensure initial color is green (stopped)
	fastled_leds[0] = CRGB(0, 30, 0);
	FastLED.show();

	// Initialize WiFi in AP mode for TCP server
	WiFi.mode(WIFI_AP);
	WiFi.softAP("KAW_Server", "password123"); // SSID and password
	IPAddress IP = WiFi.softAPIP();
	printf("AP IP address: %s\n", IP.toString().c_str());

	// Start TCP server
	tcpServer.begin();
	printf("TCP server started on port 12345\n");
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
		xTaskCreatePinnedToCore(recvTask, "RecvTask", 4096, NULL, configMAX_PRIORITIES-3, NULL, 1);
	}
	// create display queue and task
	displayQueue = xQueueCreate(4, sizeof(DisplayItem));
	if (displayQueue == NULL) {
		printf("Failed to create displayQueue\n");
	} else {
		// displayTask removed, handle in loop()
	}
	// setup()または初期化部で必ずbeaconTickerを起動
	beaconTicker.attach_ms(TDMA_FRAME_MS, [](){
		if (!beaconEnabled) return;
		if (sendQueue == NULL) return;
		SendItem item;
		item.data[0] = 0xBE; item.data[1] = 0xAC; item.len = 2;
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendFromISR(sendQueue, &item, &xHigherPriorityTaskWoken);
		if (xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
		//printf("[SERVER] Beacon sent at %lu ms\n", millis());
		//printf("%d\n", millis() - lastBeaconSentTime);
		lastBeaconSentTime = millis();
	});
}

void loop()
{
	// Check for button press to send commands
	M5.update();

	if (M5.BtnA.wasClicked()) {
		beaconEnabled = !beaconEnabled;
//		printf("beaconEnabled = %d\n", beaconEnabled);
		if (beaconEnabled) {
			sampleSeq = 0;
			fastled_leds[0] = CRGB(30, 30, 0);
		} else {
			fastled_leds[0] = CRGB(0, 30, 0);
		}
		FastLED.show();
	}
	static uint8_t prevBeaconEnabled = 0;
	if (prevBeaconEnabled != beaconEnabled) {
		if (beaconEnabled == 0) {
			xQueueReset(displayQueue);
		}
		prevBeaconEnabled = beaconEnabled;
	}
	// Handle display in loop
	DisplayItem item;
	if (xQueueReceive(displayQueue, &item, 0) == pdTRUE) {
		if (beaconEnabled) {
			char cAlternatingError = item.fAlternatingError ? '*' : ' ';
			if (fAlternatingReceiveError == 1){
				fastled_leds[0] = CRGB(30, 0, 0);
			}
			else{
				fastled_leds[0] = CRGB(30, 30, 0);
			}
			FastLED.show();
			for (int i = 0; i < 12; i++) {
				if (!beaconEnabled) break;
				printf("%c,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", cAlternatingError, item.tm/12, item.roll[i], item.pitch[i], item.yaw[i], item.roll[12+i], item.pitch[12+i], item.yaw[12+i]);
			}
		}
	}
	// TCP server handles connections asynchronously
	// No polling needed, data is received in recvTask
	delay(10);
}
