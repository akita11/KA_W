// KA_W Client
#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Ticker.h>
#include <FastLED.h>

#define ESPNOW 1
#define WIFI_UDP 2
#define WIFI_TCP 3


#define CLIENT_ID 1 // 1または2に設定（2台目は2に変更）
#define NUM_CLIENTS 2
#define WIRELESS ESPNOW
// #define WIRELESS UDP

#define CMD_START 0x01
#define CMD_STOP 0x02
#define CMD_STATUS 0x03

Ticker ticker;
#define SAMPLE_FREQ 250
#define FALLBACK_NUM_LEDS 1

// TDMA parameters (must match server)
#define TDMA_FRAME_MS 24
#define TDMA_SLOT_MS (TDMA_FRAME_MS / NUM_CLIENTS)
// Slot guard window (ms) used to decide whether to send in slot
#define TDMA_SLOT_WINDOW_MS (TDMA_SLOT_MS - 4)

volatile uint32_t last_beacon_ms = 0; // updated when beacon received
volatile uint32_t last_beacon_seq = 0; // ビーコンごとにインクリメント
volatile uint32_t last_sent_beacon_seq = 0; // 送信したビーコンのシーケンス
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

#if defined(WIRELESS) && WIRELESS == ESPNOW
#include <esp_now.h>
#endif

// Forward declarations for symbols used in recvTask/commTask
extern volatile uint8_t isTransmitting;
extern volatile uint8_t ledTransmittingState;
static void updateTransmissionLED(bool transmitting);

// Task to process received items (client)
void recvTask(void *pvParameters) {
	RecvItem item;
	for(;;) {
		if (xQueueReceive(recvQueue, &item, portMAX_DELAY) == pdTRUE) {
			uint32_t currentTime = millis();
			// Beacon detection: 2-byte beacon {0xBE,0xAC}
			if (item.len == 2 && item.data[0] == 0xBE && item.data[1] == 0xAC) {
				last_beacon_ms = currentTime;
				last_beacon_seq++;
				//printf("[CLIENT %d] Beacon received at %lu ms\n", CLIENT_ID, (unsigned long)currentTime);
				continue;
			}
			if (item.len == 1) {
				uint8_t cmd = item.data[0];
				printf("[%d ms] Command received from ", currentTime);
				for (int i = 0; i < 6; i++) {
					printf("%02X", item.mac[i]);
					if (i < 5) printf(":");
				}
				printf(": ");
				switch (cmd) {
				case CMD_START:
					printf("START\n");
					isTransmitting = 1;
					ledTransmittingState = 1;
					updateTransmissionLED(true);
					break;
				case CMD_STOP:
					printf("STOP\n");
					isTransmitting = 0;
					ledTransmittingState = 0;
					updateTransmissionLED(false);
					break;
				case CMD_STATUS:
					printf("STATUS\n");
					break;
				default:
					printf("UNKNOWN (0x%02X)\n", cmd);
					break;
				}
				continue;
			}
			// Generic data print
			printf("[%d ms] Received %d bytes from: ", currentTime, item.len);
			for (int i = 0; i < 6; i++) {
				printf("%02X", item.mac[i]);
				if (i < 5) printf(":");
			}
			printf(": ");
			for (int i = 0; i < item.len; i++) printf("%c", item.data[i]);
			printf("\n");
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

#if defined(WIRELESS) && WIRELESS == ESPNOW
#define WIRELESS_ESPNOW
#include <esp_now.h>
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
char packetBuffer[256];
int packetCount = 0;
#elif defined(WIRELESS) && WIRELESS == WIFI_TCP
#define WIRELESS_TCP
WiFiClient client;
#elif defined(WIRELESS) && WIRELESS == WIFI_UDP
#define WIRELESS_UDP
WiFiUDP udp;
#endif

#ifndef WIRELESS_ESPNOW
const char *ssid = "DualAccUDP_AP";
const char *password = "12345678";
#define PORT 12345
const char *serverIP = "192.168.4.1";
#endif

char buf[1024];
char buf_packet[1024];

volatile uint8_t isTransmitting = 0;
volatile uint8_t ledTransmittingState = 0;
volatile uint8_t lastCommandSent = 0;

static CRGB fastled_leds[FALLBACK_NUM_LEDS];
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

static void updateTransmissionLED(bool transmitting)
{
	if (!fastled_initialized)
		return;
	fastled_leds[0] = transmitting ? CRGB::Blue : CRGB::Green;
	FastLED.show();
}

volatile uint8_t fSample = 0;
void IRAM_ATTR onTicker() { fSample = 1; }

#ifdef WIRELESS_ESPNOW
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
	if (status != ESP_NOW_SEND_SUCCESS)
		printf("Last Packet Send Status = FAILED");
	//	else
	//		printf("Last Packet Send Status = SUCCESS");
	/*
	if (lastCommandSent == CMD_START)
			printf(" (command: START)\n");
		else if (lastCommandSent == CMD_STOP)
			printf(" (command: STOP)\n");
		else
			printf("\n");
	*/
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
#endif

void setup()
{
	M5.begin();
	initFastLEDFallback();
	fastled_leds[0] = CRGB(0, 30, 0);
	FastLED.show();

#ifdef WIRELESS_ESPNOW
	WiFi.mode(WIFI_MODE_STA);
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
#else
	WiFi.begin(ssid, password);
	printf("Connecting to WiFi...");
	int attempts = 0;
	while (WiFi.status() != WL_CONNECTED && attempts++ < 20)
	{
		delay(500);
		printf(".");
	}
	if (WiFi.status() == WL_CONNECTED)
		printf("\nConnected to WiFi, IP: %s\n", WiFi.localIP().toString());
#ifdef WIRELESS_TCP
	printf("Connecting to TCP server at %s:%d...\n", serverIP, PORT);
	int maxAttempts = 5;
	while (!client.connect(serverIP, PORT) && maxAttempts-- > 0)
	{
		printf(".");
		delay(1000);
	}
	if (client.connected())
		printf("\nConnected to TCP server\n");
	else
		printf("\nFailed to connect to TCP server\n");
#endif
#endif

#ifdef WIRELESS_ESPNOW
	packetCount = 0;
	memset(packetBuffer, 0, sizeof(packetBuffer));
#endif
	ticker.attach_ms((int)(1000 / SAMPLE_FREQ), onTicker);
	ledTransmittingState = isTransmitting;
	updateTransmissionLED(isTransmitting);
	strcpy(buf_packet, "");
}

uint16_t n = 0;
#define DATA_PER_PACKET 6
// memo: ESPNOWは256バイトまで?

uint32_t lastReceiveTime, currentTime;


// 4msごとにダミーデータをバッファし、TDMAスロット進入時に最新から6個分まとめて送信
char sample_buf[6][64];
uint8_t sample_count = 0;

void loop()
{
	fSample = 0;
	while (fSample == 0)
		delayMicroseconds(10);

	// 1未満・小数点以下3桁の実数6個をコンマ区切りで生成
	float v[6];
	for (int i = 0; i < 6; i++) {
		v[i] = ((float)(rand() % 1000)) / 1000.0f; // 0.000～0.999
	}
	sample_count = (sample_count + 1) % 6;
	sprintf(sample_buf[sample_count], "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", v[0], v[1], v[2], v[3], v[4], v[5]);

	if (isTransmitting == 1) {
		uint32_t now = millis();
		if (last_beacon_ms == 0) return;
		uint32_t offset = (now - last_beacon_ms) % TDMA_FRAME_MS;
		uint32_t my_slot_start = (CLIENT_ID - 1) * TDMA_SLOT_MS;
		if (offset >= my_slot_start && offset < my_slot_start + TDMA_SLOT_WINDOW_MS) {
			if (last_sent_beacon_seq != last_beacon_seq) {
				//printf("[CLIENT %d] Enter TDMA slot at %lu ms (offset=%lu, slot_start=%lu)\n", CLIENT_ID, (unsigned long)now, (unsigned long)offset, (unsigned long)my_slot_start);
				// 最新データから遡って6個分まとめて送信
				char sendbuf[400] = "";
				for (int i = 0; i < 6; i++) {
					int idx = (sample_count + 6 - i) % 6;
					strcat(sendbuf, sample_buf[idx]);
					strcat(sendbuf, "\n");
				}
#ifdef WIRELESS_ESPNOW
				if (!esp_now_is_peer_exist(broadcastAddress)) {
					esp_now_peer_info_t peer = {};
					memcpy(peer.peer_addr, broadcastAddress, 6);
					peer.channel = 0;
					peer.encrypt = false;
					esp_now_add_peer(&peer);
				}
				//printf("[CLIENT %d] TDMA SEND: %d bytes\n", CLIENT_ID, (int)strlen(sendbuf));
				size_t msglen = strlen(sendbuf) + 1;
				if (msglen > 250) msglen = 250; // ESPNOW最大ペイロード制限
				if (sendQueue != NULL) {
					SendItem item;
					memcpy(item.addr, broadcastAddress, 6);
					memcpy(item.data, sendbuf, msglen);
					item.len = msglen;
					if (xQueueSend(sendQueue, &item, pdMS_TO_TICKS(10)) != pdTRUE) {
						printf("client: sendQueue full - drop packet\n");
					}
				} else {
					if (esp_now_send(broadcastAddress, (uint8_t *)sendbuf, msglen) != ESP_OK)
						printf("Error sending data\n");
				}
#endif
				last_sent_beacon_seq = last_beacon_seq;
			}
		}
	}
}
