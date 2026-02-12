// KA_W Client
#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Ticker.h>
#include <FastLED.h>
#include <Preferences.h>
#include "imu.h"

// Note: q0-q3 in MadgwickAHRS.h must be moved from private to public
// memo: q0=scalar, q1,q2,q3=vector

#define CLIENT_ID 1 // 1 or 2

//#define USE_TCP

//#define DEBUG

Ticker ticker;
#define SAMPLE_FREQ 250
#define STAGGER_MS 0 // per-client start offset in ms (CLIENT_ID 1 -> 0ms, 2 -> STAGGER_MS)
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
#define N_SAMPLE_CALIB (SAMPLE_FREQ * 30) // 30[s]
uint16_t nSampleCalib = 0;

// START-controlled operation
volatile bool fStarted = false; // set when START received from server
volatile uint32_t last_start_ms = 0;
volatile uint32_t last_send_ms = 0; // last time we sent our periodic payload
volatile uint32_t delayed_start_until = 0; // if non-zero, wait until this millis() before first send
volatile uint32_t start_offset_ms = 0;
// Debug-print macro: enable when DEBUG is defined
#ifndef DBG_PRINT
#ifdef DEBUG
#define DBG_PRINT(...) printf(__VA_ARGS__)
#else
#define DBG_PRINT(...) do{} while(0)
#endif
#endif
// Communication task and queue for client
typedef struct
{
	uint8_t data[256];
	size_t len;
} SendItem;

static QueueHandle_t sendQueue = NULL;
static TaskHandle_t commTaskHandle = NULL;

// UDP client
WiFiUDP udp;
const uint16_t UDP_PORT = 12345;
#if defined(USE_TCP)
WiFiServer tcpServer(UDP_PORT);
#endif

Madgwick madgwick;

Preferences prefs;

// forward declare LED array (defined later)
extern CRGB fastled_leds[];

// Process START/STOP commands received from server
static void handleReceivedCommand(uint8_t *data, int len)
{
	uint32_t currentTime = millis();
	// START detection: 2-byte START {0xAA,0x01}
	if (len == 2 && data[0] == 0xAA && data[1] == 0x01)
	{
		fStarted = true;
		last_start_ms = currentTime;
		start_offset_ms = (uint32_t)((CLIENT_ID > 0) ? (CLIENT_ID - 1) * STAGGER_MS : 0);
		delayed_start_until = currentTime + start_offset_ms;
		last_send_ms = 0;
		fastled_leds[0] = CRGB(0, 0, 30);
		FastLED.show();
		DBG_PRINT("[CLIENT %d] START received\n", CLIENT_ID);
		return;
	}
	// STOP detection: 2-byte STOP {0xBB,0x00}
	if (len == 2 && data[0] == 0xBB && data[1] == 0x00)
	{
		fStarted = false;
		delayed_start_until = 0;
		start_offset_ms = 0;
		fastled_leds[0] = CRGB(0, 30, 0);
		FastLED.show();
		DBG_PRINT("[CLIENT %d] STOP received\n", CLIENT_ID);
		return;
	}
}

void recvTask(void *pvParameters)
{
	for (;;)
	{
#if defined(USE_TCP)
		WiFiClient c = tcpServer.available();
		if (c)
		{
			uint8_t buf[256];
			int len = c.read(buf, sizeof(buf));
			if (len > 0)
				handleReceivedCommand(buf, len);
			c.stop();
		}
#else
		int packetSize = udp.parsePacket();
		if (packetSize > 0)
		{
			uint8_t buf[256];
			int len = udp.read((char *)buf, sizeof(buf));
			if (len > 0)
				handleReceivedCommand(buf, len);
		}
#endif
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

void commTask(void *pvParameters)
{
	SendItem item;
	for (;;)
	{
		if (xQueueReceive(sendQueue, &item, portMAX_DELAY) == pdTRUE)
		{
			// send to server via UDP or TCP depending on compile-time flag
			IPAddress serverIP(192, 168, 4, 1);
#if defined(USE_TCP)
			WiFiClient tcp;
			if (tcp.connect(serverIP, UDP_PORT))
			{
				tcp.write(item.data, item.len);
				tcp.stop();
			}
			else
			{
				DBG_PRINT("client: TCP connect failed to %s\n", serverIP.toString().c_str());
			}
#else
			udp.beginPacket(serverIP, UDP_PORT);
			udp.write(item.data, item.len);
			udp.endPacket();
#endif
			vTaskDelay(pdMS_TO_TICKS(1));
		}
	}
}

CRGB fastled_leds[FALLBACK_NUM_LEDS];
static bool fastled_initialized = false;

static void initFastLEDFallback()
{
	if (fastled_initialized)
		return;
	FastLED.addLeds<WS2812B, 35, GRB>(fastled_leds, FALLBACK_NUM_LEDS);
	FastLED.setBrightness(64);
	fastled_initialized = true;
	fastled_leds[0] = CRGB(0, 30, 0);
	FastLED.show();
}

volatile uint8_t fSample = 0;
char sample_buf[12][21]; // 1バッファ=20文字+1(終端)
uint8_t sample_count = 0;
uint8_t sampleSeq = 0;

volatile uint32_t micros0 = 0;

void IRAM_ATTR onTicker()
{
	uint8_t buf[20];
	// BMI270 acc/gyroデータレジスタ: 0x0C～0x1F
	if (readReg(I2C_ADDR_IMU, BMI270_REG_AUX_DATA, buf, 20))
	{
		// Process IMU data
		axRaw = conv_value(buf[9], buf[8]);
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
		if (stCalibrate == CALIB_STATE_STARTED)
		{
			gxSum += gxRaw;
			gySum += gyRaw;
			gzSum += gzRaw;
			nSampleCalib++;
			if (nSampleCalib == N_SAMPLE_CALIB)
			{
				stCalibrate = CALIB_STATE_DONE;
				gxOffset = (float)(gxSum / N_SAMPLE_CALIB) / 32768.0f * GYRO_MAX_DPS; // [dps]
				gyOffset = (float)(gySum / N_SAMPLE_CALIB) / 32768.0f * GYRO_MAX_DPS; // [dps]
				gzOffset = (float)(gzSum / N_SAMPLE_CALIB) / 32768.0f * GYRO_MAX_DPS; // [dps]
				printf("Calibration done: gxOffset=%.3f, gyOffset=%.3f, gzOffset=%.3f\n", gxOffset, gyOffset, gzOffset);
			}
		}
		gx -= gxOffset;
		gy -= gyOffset;
		gz -= gzOffset;
	}
	fSample = 1;
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

	// IMUの初期化（エラーチェック付き）
	fastled_leds[0] = CRGB(30, 0, 0); // 初期化中は赤
	FastLED.show();

	while (IMUinit(I2C_ADDR_IMU) != BMI270_OK)
		delay(10);

	fastled_leds[0] = CRGB(0, 30, 0); // green
	FastLED.show();

	madgwick.begin(SAMPLE_FREQ);

	ticker.attach_ms((int)(1000 / SAMPLE_FREQ), onTicker);

	// Check button for calibration
	prefs.begin("imu_offsets", false); // namespace "imu_offsets", read/write
	if (M5.BtnA.isPressed()) {
		// Button pressed: perform calibration
		DBG_PRINT("[CLIENT %d] Button pressed, starting calibration...\n", CLIENT_ID);
		stCalibrate = CALIB_STATE_STARTED;
		gxSum = 0;
		gySum = 0;
		gzSum = 0;
		gxOffset = 0;
		gyOffset = 0;
		gzOffset = 0;
		nSampleCalib = 0;
		fastled_leds[0] = CRGB(30, 20, 0); // orange, calibrating
		FastLED.show();
		// Wait for calibration to complete
		while (stCalibrate != CALIB_STATE_DONE) {
			delay(10);
		}
		fastled_leds[0] = CRGB::Black; // turn off after calibration
		FastLED.show();
		// Save offsets to EEPROM
		prefs.putFloat("gxOffset", gxOffset);
		prefs.putFloat("gyOffset", gyOffset);
		prefs.putFloat("gzOffset", gzOffset);
		DBG_PRINT("[CLIENT %d] Offsets saved to EEPROM\n", CLIENT_ID);
	} else {
		// Button not pressed: load offsets from EEPROM
		gxOffset = prefs.getFloat("gxOffset", 0.0f);
		gyOffset = prefs.getFloat("gyOffset", 0.0f);
		gzOffset = prefs.getFloat("gzOffset", 0.0f);
		stCalibrate = CALIB_STATE_DONE;
		DBG_PRINT("[CLIENT %d] Offsets loaded from EEPROM: gx=%.3f, gy=%.3f, gz=%.3f\n\n", CLIENT_ID, gxOffset, gyOffset, gzOffset);
	}
	prefs.end();

	// Connect to WiFi AP with static IP (no DHCP)
	uint8_t lastOctet = 1 + CLIENT_ID; // e.g. CLIENT_ID=1 -> 192.168.4.2
	IPAddress localIP(192, 168, 4, lastOctet);
	IPAddress gateway(192, 168, 4, 1);
	IPAddress subnet(255, 255, 255, 0);
	if (!WiFi.config(localIP, gateway, subnet)) {
		DBG_PRINT("[CLIENT %d] Failed to configure static IP\n", CLIENT_ID);
	} else {
		DBG_PRINT("[CLIENT %d] Static IP configured: %s\n", CLIENT_ID, localIP.toString().c_str());
	}
	WiFi.begin("KAW_Server", "password123");
	DBG_PRINT("[CLIENT %d] Starting WiFi connection to KAW_Server...\n", CLIENT_ID);
	// Blink blue while waiting for WiFi to connect (500ms period)
	bool wifiBlink = false;
	uint32_t connectStart = millis();
	uint32_t timeoutMs = 30000; // 30 seconds timeout
	while (WiFi.status() != WL_CONNECTED) {
		wifiBlink = !wifiBlink;
		fastled_leds[0] = wifiBlink ? CRGB::Blue : CRGB::Black;
		FastLED.show();
		DBG_PRINT("[CLIENT %d] WiFi status: %d (0=IDLE, 1=NO_SSID, 3=CONNECTED, 4=FAILED, 6=DISCONNECTED)\n", CLIENT_ID, WiFi.status());
		delay(500);
		if (millis() - connectStart > timeoutMs) {
			DBG_PRINT("[CLIENT %d] WiFi connection timeout after %d ms\n", CLIENT_ID, timeoutMs);
			break;
		}
	}
	if (WiFi.status() == WL_CONNECTED) {
		DBG_PRINT("[CLIENT %d] Connected to WiFi, IP=%s, RSSI=%d dBm\n", CLIENT_ID, WiFi.localIP().toString().c_str(), WiFi.RSSI());
		fastled_leds[0] = CRGB(0, 30, 0);
		FastLED.show();
	} else {
		DBG_PRINT("[CLIENT %d] Failed to connect to WiFi\n", CLIENT_ID);
		fastled_leds[0] = CRGB::Red;
		FastLED.show();
	}

	// Start network listener depending on transport
#if defined(USE_TCP)
	tcpServer.begin();
	DBG_PRINT("TCP server started on port %d\n", UDP_PORT);
#else
	if (!udp.begin(UDP_PORT))
	{
		DBG_PRINT("Failed to start UDP on port %d\n", UDP_PORT);
		for (;;)
		{
			fastled_leds[0] = CRGB::Red;
			FastLED.show();
			delay(200);
			fastled_leds[0] = CRGB::Black;
			FastLED.show();
			delay(200);
		}
	}
	else
	{
		DBG_PRINT("UDP ready on port %d\n", UDP_PORT);
	}
#endif
	// create send queue and communication task (pinned to core 0)
	sendQueue = xQueueCreate(8, sizeof(SendItem));
	if (sendQueue == NULL)
	{
		DBG_PRINT("client: Failed to create sendQueue\n");
	}
	else
	{
		xTaskCreatePinnedToCore(commTask, "CommTask", 4096, NULL, configMAX_PRIORITIES - 2, &commTaskHandle, 0);
	}
	// create receive task
	xTaskCreatePinnedToCore(recvTask, "RecvTask", 4096, NULL, configMAX_PRIORITIES - 3, NULL, 0);
}

void loop()
{
	fSample = 0;
	while (fSample == 0)
		delayMicroseconds(10);

	if (stCalibrate == CALIB_STATE_DONE)
	{
		madgwick.updateIMU(gx, gy, gz, ax, ay, az);
	}

	// 1000倍・6桁ゼロ埋め整数化
	int iq0 = (int)roundf(madgwick.q0 * 1000.0f);
	int iq1 = (int)roundf(madgwick.q1 * 1000.0f);
	int iq2 = (int)roundf(madgwick.q2 * 1000.0f);
	int iq3 = (int)roundf(madgwick.q3 * 1000.0f);
	// 各q(sign+i1桁+f3桁)（合計20文字）
	snprintf(sample_buf[sample_count], sizeof(sample_buf[0]), "%+05d%+05d%+05d%+05d", iq0, iq1, iq2, iq3);
	sample_count = (sample_count + 1) % 12;
	// If not started by SERVER, do nothing
	if (!fStarted)
		return;
	// Send only when 12 samples have been collected (sample_count wrapped to 0)
	if (stCalibrate == CALIB_STATE_DONE && sample_count == 0)
	{
		// If a staggered start is configured, wait until the scheduled time for the first send
		if (delayed_start_until != 0 && millis() < delayed_start_until)
			return;

		char sendbuf[400] = "";
		sprintf(sendbuf, "%c", '0' + CLIENT_ID);
		for (int i = 0; i < 12; i++)
		{
			int idx = (sample_count + i) % 12;
			strcat(sendbuf, sample_buf[idx]);
		}
		strcat(sendbuf, "\n");
		// Log elapsed ms since previous send (0 for first send)
		unsigned long now_ms = millis();
		unsigned long elapsed_ms = (last_send_ms == 0) ? 0 : (now_ms - last_send_ms);
		DBG_PRINT("[CLIENT %d] send elapsed %lu ms\n", CLIENT_ID, elapsed_ms);
		size_t msglen = strlen(sendbuf) + 1;
		if (msglen > 250)
			msglen = 250; // UDP payload limit
		if (sendQueue != NULL)
		{
			SendItem item;
			memcpy(item.data, sendbuf, msglen);
			item.len = msglen;
			if (xQueueSend(sendQueue, &item, pdMS_TO_TICKS(10)) != pdTRUE)
			{
				DBG_PRINT("client: sendQueue full - drop packet\n");
			}
		}
		last_send_ms = now_ms;
	}
}
