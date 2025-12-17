// KA_W Client
#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Ticker.h>
#include <FastLED.h>

Ticker ticker;
#define SAMPLE_FREQ 250
#define FALLBACK_NUM_LEDS 1
#define CALIB_STATE_NONE 0
#define CALIB_STATE_STARTED 1
#define CALIB_STATE_DONE 2
uint8_t stCalibrate = CALIB_STATE_NONE;

// TDMA parameters (must match server)
#define TDMA_FRAME_MS 48
#define TDMA_SLOT_MS (TDMA_FRAME_MS / NUM_CLIENTS)
// Slot guard window (ms) used to decide whether to send in slot
#define TDMA_SLOT_WINDOW_MS (TDMA_SLOT_MS - 4)

volatile uint32_t last_beacon_ms = 0; // updated when beacon received
volatile uint32_t last_beacon_seq = 0; // ビーコンごとにインクリメント
volatile uint32_t last_sent_beacon_seq = 0; // 送信したビーコンのシーケンス
volatile uint32_t expected_beacon_seq = 0; // 期待されるビーコンシーケンス
volatile bool beacon_lost = false; // ビーコン受信ミスフラグ
// Communication task and queue for client
typedef struct {
	uint8_t addr[6];
	uint8_t data[256];
	size_t len;
} SendItem;

static QueueHandle_t sendQueue = NULL;
static TaskHandle_t commTaskHandle = NULL;

// Received data queue and task
typedef struct {
	uint8_t mac[6];
	uint8_t data[256];
	int len;
} RecvItem;

static QueueHandle_t recvQueue = NULL;

#include <esp_now.h>

// Task to process received items (client)
void recvTask(void *pvParameters) {
	RecvItem item;
	for(;;) {
		if (xQueueReceive(recvQueue, &item, portMAX_DELAY) == pdTRUE) {
			uint32_t currentTime = millis();
			// Beacon detection: 2-byte beacon {0xBE,0xAC}
			if (item.len == 2 && item.data[0] == 0xBE && item.data[1] == 0xAC) {
				uint32_t interval = (last_beacon_ms == 0) ? 0 : (currentTime - last_beacon_ms);
				printf("Beacon interval: %d ms / 48ms), Seq: %d", interval, last_beacon_seq + 1);
				bool hasLost = false;
				if (expected_beacon_seq > 0 && last_beacon_seq + 1 != expected_beacon_seq) {
					uint32_t lost = (last_beacon_seq + 1) - expected_beacon_seq;
					printf(" (Lost: %d)", lost);
					hasLost = true;
				}
				beacon_lost = hasLost;
				last_beacon_ms = currentTime;
				last_beacon_seq++;
				expected_beacon_seq = last_beacon_seq + 1;
				continue;
			}
		}
	}
}

void commTask(void *pvParameters) {
	SendItem item;
	for(;;) {
		if (xQueueReceive(sendQueue, &item, portMAX_DELAY) == pdTRUE) {
			esp_err_t res = esp_now_send(item.addr, item.data, item.len);
			if (res != ESP_OK) {
				printf("client commTask: esp_now_send failed %d\n", res);
			}
			vTaskDelay(pdMS_TO_TICKS(1));
		}
	}
}

#include <esp_now.h>
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
char packetBuffer[256];
int packetCount = 0;

char buf[1024];
char buf_packet[1024];

CRGB fastled_leds[FALLBACK_NUM_LEDS];
static bool fastled_initialized = false;
static int fastled_pin = -1;

static void initFastLEDFallback()
{
	if (fastled_initialized)
		return;
	fastled_pin = 35;
	FastLED.addLeds<WS2812B, 35, GRB>(fastled_leds, FALLBACK_NUM_LEDS);
	FastLED.setBrightness(64);
	fastled_initialized = true;
	fastled_leds[0] = CRGB::Green;
	FastLED.show();
}

volatile uint32_t micros0 = 0;

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
	if (status != ESP_NOW_SEND_SUCCESS)
		printf("Last Packet Send Status = FAILED");
	//	else
	//		printf("Last Packet Send Status = SUCCESS");
}

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
	// enqueue received data to recvQueue for processing in recvTask
	if (recvQueue == NULL) return;
	RecvItem item;
	memcpy(item.mac, mac, 6);
	int copyLen = len;
	if (copyLen > (int)sizeof(item.data)) copyLen = sizeof(item.data);
	memcpy(item.data, incomingData, copyLen);
	item.len = copyLen;

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	BaseType_t ok = xQueueSendFromISR(recvQueue, &item, &xHigherPriorityTaskWoken);
	if (ok != pdTRUE) {
		ok = xQueueSend(recvQueue, &item, 0);
	}
	if (xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}

void setup()
{
	auto cfg = M5.config();
	cfg.external_imu = false;
	cfg.internal_imu = false;
	cfg.internal_spk = false;
	cfg.internal_mic = false;

	// I2C clock is defined in Unified/src/utility/imu/IMU_Base.hpp
	M5.begin(cfg);
	M5.Ex_I2C.begin();

	initFastLEDFallback();
	fastled_leds[0] = CRGB(0, 30, 0);
	FastLED.show();

	WiFi.mode(WIFI_MODE_STA);
	WiFi.channel(1); // チャンネルを固定して通信安定化
	if (esp_now_init() != ESP_OK)
	{
		printf("Error initializing ESP-NOW\n");
		return;
	}
	esp_now_register_send_cb(onDataSent);
	esp_now_register_recv_cb(onDataRecv);
	esp_now_peer_info_t peerInfo = {};
	memcpy(peerInfo.peer_addr, broadcastAddress, 6);
	peerInfo.channel = 0;
	peerInfo.encrypt = false;
	if (esp_now_add_peer(&peerInfo) != ESP_OK)
	{
		printf("Failed to add peer\n");
		return;
	}
	printf("ESP-NOW initialized in STA mode (sending)\nBroadcast peer added\n");
	// create send queue and communication task (pinned to core 0)
	sendQueue = xQueueCreate(8, sizeof(SendItem));
	if (sendQueue == NULL) {
		printf("client: Failed to create sendQueue\n");
	} else {
		xTaskCreatePinnedToCore(commTask, "CommTask", 4096, NULL, configMAX_PRIORITIES-2, &commTaskHandle, 0);
	}
	// create receive queue and task
	recvQueue = xQueueCreate(16, sizeof(RecvItem));
	if (recvQueue == NULL) {
		printf("client: Failed to create recvQueue\n");
	} else {
		xTaskCreatePinnedToCore(recvTask, "RecvTask", 4096, NULL, configMAX_PRIORITIES-3, NULL, 0);
	}

	packetCount = 0;
	memset(packetBuffer, 0, sizeof(packetBuffer));
	strcpy(buf_packet, "");
}

uint32_t lastReceiveTime, currentTime;

void loop()
{
	// beacon: every 48ms
#define BEACON_CHECK_CYCLE 6
	uint8_t cnt = last_beacon_seq % BEACON_CHECK_CYCLE; // cycle=288ms (3.47Hz)
	if ((last_beacon_seq % BEACON_CHECK_CYCLE) < BEACON_CHECK_CYCLE) fastled_leds[0] = CRGB(0, 30, 0);
	else fastled_leds[0] = CRGB(0, 0, 30);
	FastLED.show();
}
