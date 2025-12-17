// KA_W Client
#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Ticker.h>
#include <FastLED.h>
#include "imu.h"

#define CLIENT_ID 1 // 1 or 2
#define NUM_CLIENTS 2

Ticker ticker;
#define SAMPLE_FREQ 250
#define FALLBACK_NUM_LEDS 1
#define CALIB_STATE_NONE 0
#define CALIB_STATE_STARTED 1
#define CALIB_STATE_DONE 2
uint8_t stCalibrate = CALIB_STATE_NONE;

float ax, ay, az;
float gx, gy, gz;
volatile int axRaw, ayRaw, azRaw;
volatile int gxRaw, gyRaw, gzRaw;
float gxOffset, gyOffset, gzOffset;
long gxSum, gySum, gzSum;
#define N_SAMPLE_CALIB (SAMPLE_FREQ * 4) // 4[s]
//#define N_SAMPLE_CALIB (SAMPLE_FREQ * 10) // 10[s]
uint16_t nSampleCalib = 0;

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

Madgwick madgwick;

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
				//printf("[CLIENT %d] Interval: %d ms, Seq: %d", CLIENT_ID, interval, last_beacon_seq + 1);
				if (stCalibrate == CALIB_STATE_NONE){
					stCalibrate = CALIB_STATE_STARTED;
					gxSum = 0; gySum = 0; gzSum = 0;
					gxOffset = 0; gyOffset = 0; gzOffset = 0;
					//printf(" - Calibration started");
				}
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
			/*
			// Generic data print
			printf("[%d ms] Received %d bytes from: ", currentTime, item.len);
			for (int i = 0; i < 6; i++) {
				printf("%02X", item.mac[i]);
				if (i < 5) printf(":");
			}
			printf(": ");
			for (int i = 0; i < item.len; i++) printf("%c", item.data[i]);
			printf("\n");
			*/
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
	ticker.attach_ms((int)(1000 / SAMPLE_FREQ), onTicker);
	strcpy(buf_packet, "");
}

uint16_t n = 0;
// memo: ESPNOWは256バイトまで

uint32_t lastReceiveTime, currentTime;

// 4msごとにIMU+MadgwickでYaw/Roll/Pitch（1000倍6桁整数×3連結）をバッファし、TDMAスロット進入時に最新から12個分まとめて送信
char sample_buf[12][20]; // 1バッファ=18文字+1(終端)
uint8_t sample_count = 0;
uint8_t sampleSeq = 0;

volatile uint32_t micros0 = 0;

void loop()
{
	float roll, pitch, yaw;
	// LED制御: ビーコン受信ミスがあれば赤、なければ緑
	if (beacon_lost) {
		fastled_leds[0] = CRGB(30, 0, 0); // 赤
	} else {
		fastled_leds[0] = CRGB(0, 30, 0); // 緑
	}
	FastLED.show();

	fSample = 0;
	while (fSample == 0)
		delayMicroseconds(10);

	uint8_t buf[20];
	// BMI270 acc/gyroデータレジスタ: 0x0C～0x1F
	if (readReg(I2C_ADDR_IMU, BMI270_REG_AUX_DATA, buf, 20)){
		// Process IMU data
		axRaw = conv_value(buf[ 9], buf[ 8]);
		ayRaw = conv_value(buf[11], buf[10]);
		azRaw = conv_value(buf[13], buf[12]);
		gxRaw = conv_value(buf[15], buf[14]);
		gyRaw = conv_value(buf[17], buf[16]);
		gzRaw = conv_value(buf[19], buf[18]);
		ax = (float)axRaw / 32768.0f * ACC_MAX_MS2; // [g]
		ay = (float)ayRaw / 32768.0f * ACC_MAX_MS2;
		az = (float)azRaw / 32768.0f * ACC_MAX_MS2;
		gx = (float)gxRaw / 32768.0f * GYRO_MAX_DPS; // [dps]
		gy = (float)gyRaw / 32768.0f * GYRO_MAX_DPS;
		gz = (float)gzRaw / 32768.0f * GYRO_MAX_DPS;
		if (stCalibrate == CALIB_STATE_STARTED) {
			fastled_leds[0] = CRGB(30, 10, 0); FastLED.show();
			//printf("calibrating... %d / %d\r", nSampleCalib, N_SAMPLE_CALIB);
			gxSum += gxRaw;
			gySum += gyRaw;
			gzSum += gzRaw;
			nSampleCalib++;
			if (nSampleCalib == N_SAMPLE_CALIB){
				fastled_leds[0] = CRGB(0, 30, 0); FastLED.show();
				stCalibrate = CALIB_STATE_DONE;
				gxOffset = (float)(gxSum / N_SAMPLE_CALIB) / 32768.0f * GYRO_MAX_DPS; // [dps]
				gyOffset = (float)(gySum / N_SAMPLE_CALIB	) / 32768.0f * GYRO_MAX_DPS; // [dps]
				gzOffset = (float)(gzSum / N_SAMPLE_CALIB) / 32768.0f * GYRO_MAX_DPS; // [dps]
				//printf("Calibration done: gxOffset=%.3f, gyOffset=%.3f, gzOffset=%.3f\n", gxOffset, gyOffset, gzOffset);
			}
		}
		gx -= gxOffset;
		gy -= gyOffset;
		gz -= gzOffset;
		if (stCalibrate == CALIB_STATE_DONE) {
			madgwick.updateIMU(gx, gy, gz, ax, ay, az);
			roll = madgwick.getRoll();   // degree
			pitch = madgwick.getPitch(); // degree
			yaw = madgwick.getYaw();     // degree
			// test dummy data
			yaw = (float)sampleSeq + 0.123;
			roll = (float)sampleSeq + 0.456;
			pitch = (float)sampleSeq + 0.789;
			sampleSeq++;
			// end of test dummy data
			//uint32_t t = micros();
			//printf("%d %.3f %3f %.3f\n", t - micros0, yaw, roll, pitch);
			//micros0 = t;
		}
	} else {
		roll = pitch = yaw = 0.0f;
	}

	while(roll < 0.0f) roll += 360.0f;
	while(yaw < 0.0f) yaw += 360.0f;
	while(pitch < 0.0f) pitch += 360.0f;

	// 1000倍・6桁ゼロ埋め整数化
	int iroll = (int)roundf(roll * 1000.0f);
	int ipitch = (int)roundf(pitch * 1000.0f);
	int iyaw = (int)roundf(yaw * 1000.0f);
	// 6桁Yaw + 6桁Roll + 6桁Pitch（合計18文字, 先頭0埋め）
	if (sample_count == 0){
		uint32_t t = micros();
		printf("%d %d\n", t - micros0, sample_count);
		micros0 = t;
	}
	snprintf(sample_buf[sample_count], sizeof(sample_buf[0]), "%06d%06d%06d", iyaw, iroll, ipitch);
	/*
	// --- Teleplot形式で10サンプルに1回のみ出力 ---
	static int teleplot_counter = 0;
	teleplot_counter++;
	if (teleplot_counter >= 10) {
		teleplot_counter = 0;
		// Teleplot形式: "teleplot:変数名 値"
		printf("teleplot:Yaw %.3f\n", yaw);
		printf("teleplot:Roll %.3f\n", roll);
		printf("teleplot:Pitch %.3f\n", pitch);
//		printf("%d %.3f %.3f %.3f / %.3f %.3f %.3f / %.1f %.1f %.1f\n", millis() % 1000, ax, ay, az, gx, gy, gz, yaw, roll, pitch);
//		printf("%d %.1f %.1f %.1f\n", millis() % 1000, yaw, roll, pitch);
//		printf("%s\n", sample_buf[sample_count]);
	}
	*/
	sample_count = (sample_count + 1) % 12;

	if (last_beacon_ms == 0) return;
	uint32_t now = millis();
	uint32_t offset = (now - last_beacon_ms) % TDMA_FRAME_MS;
	uint32_t my_slot_start = (CLIENT_ID - 1) * TDMA_SLOT_MS;
	if (offset >= my_slot_start && offset < my_slot_start + TDMA_SLOT_WINDOW_MS) {
		if (last_sent_beacon_seq != last_beacon_seq) {
			if (stCalibrate == CALIB_STATE_DONE){
				char sendbuf[400] = "";
				sprintf(sendbuf, "%c", '0' + CLIENT_ID);
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
					printf("%d %d %s\n", sample_count, now, sendbuf);
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
}
