
// KA_W Client
#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Ticker.h>
#include <FastLED.h>
#include "imu.h"

#define CLIENT_ID 1 // 1または2に設定（2台目は2に変更）
#define NUM_CLIENTS 2

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

#include <esp_now.h>

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

#include <esp_now.h>
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
char packetBuffer[256];
int packetCount = 0;

char buf[1024];
char buf_packet[1024];

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

volatile uint8_t fSample = 0;
void IRAM_ATTR onTicker() { fSample = 1; }

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

	// IMUの初期化（エラーチェック付き）
	fastled_leds[0] = CRGB(30, 0, 0); // 初期化中は赤
	FastLED.show();

	int result;
	while(IMUinit(I2C_ADDR_IMU) != BMI270_OK) delay(10);
	if (result != BMI270_OK)
	{
		while (1)
		{
			fastled_leds[0] = CRGB::Red; FastLED.show(); delay(200);
			fastled_leds[0] = CRGB::Black; FastLED.show(); delay(200);
		}
	}

	fastled_leds[0] = CRGB(0, 30, 0); // green
	FastLED.show();

	madgwick.begin(SAMPLE_FREQ);

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

	packetCount = 0;
	memset(packetBuffer, 0, sizeof(packetBuffer));
	ticker.attach_ms((int)(1000 / SAMPLE_FREQ), onTicker);
	strcpy(buf_packet, "");
}

uint16_t n = 0;
// memo: ESPNOWは256バイトまで

uint32_t lastReceiveTime, currentTime;

// 4msごとにIMU+MadgwickでYaw/Roll/Pitch（1000倍6桁整数×3連結）をバッファし、TDMAスロット進入時に最新から12個分まとめて送信
char sample_buf[12][20]; // 1バッファ=18文字+1(終端)
uint8_t sample_count = 0;

void loop()
{
	fSample = 0;
	while (fSample == 0)
		delayMicroseconds(10);

	// IMUサンプリング・Madgwickフィルタ適用
	if (readIMU(ax, ay, az, gx, gy, gz)) {
		madgwick.updateIMU(gx, gy, gz, ax, ay, az);
		roll = madgwick.getRoll();   // degree
		pitch = madgwick.getPitch(); // degree
		yaw = madgwick.getYaw();     // degree
	} else {
		roll = pitch = yaw = 0.0f;
	}

	// 1000倍・6桁ゼロ埋め整数化
	int iroll = (int)roundf(roll * 1000.0f);
	int ipitch = (int)roundf(pitch * 1000.0f);
	int iyaw = (int)roundf(yaw * 1000.0f);
	char id = '0' + CLIENT_ID;
	// 1桁ID + 6桁Yaw + 6桁Roll + 6桁Pitch（合計19文字, 先頭0埋め）
	snprintf(sample_buf[sample_count], sizeof(sample_buf[0]), "%c%06d%06d%06d", id, iyaw, iroll, ipitch);

	// --- Teleplot形式で10サンプルに1回のみ出力 ---
	static int teleplot_counter = 0;
	teleplot_counter++;
	if (teleplot_counter >= 10) {
		teleplot_counter = 0;
/*
		// Teleplot形式: "teleplot:変数名 値"
		printf("teleplot:Yaw %.3f\n", yaw);
		printf("teleplot:Roll %.3f\n", roll);
		printf("teleplot:Pitch %.3f\n", pitch);
		*/
//		printf("%d %.3f %.3f %.3f / %.3f %.3f %.3f / %.1f %.1f %.1f\n", millis() % 1000, ax, ay, az, gx, gy, gz, yaw, roll, pitch);
		printf("%d %.1f %.1f %.1f\n", millis() % 1000, yaw, roll, pitch);
	}

	sample_count = (sample_count + 1) % 12;

	if (last_beacon_ms == 0) return;
	uint32_t now = millis();
	uint32_t offset = (now - last_beacon_ms) % TDMA_FRAME_MS;
	uint32_t my_slot_start = (CLIENT_ID - 1) * TDMA_SLOT_MS;
	if (offset >= my_slot_start && offset < my_slot_start + TDMA_SLOT_WINDOW_MS) {
		if (last_sent_beacon_seq != last_beacon_seq) {
			char sendbuf[400] = "";
			for (int i = 0; i < 12; i++) {
				int idx = (sample_count + i) % 12;
				strcat(sendbuf, sample_buf[idx]);
			}
			size_t msglen = strlen(sendbuf) + 1;
			if (msglen > 250) msglen = 250; // ESPNOW最大ペイロード制限
			if (!esp_now_is_peer_exist(broadcastAddress)) {
				esp_now_peer_info_t peer = {};
				memcpy(peer.peer_addr, broadcastAddress, 6);
				peer.channel = 0;
				peer.encrypt = false;
				esp_now_add_peer(&peer);
			}
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
			last_sent_beacon_seq = last_beacon_seq;
		}
	}
}
