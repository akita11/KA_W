// KA_W Server
// Configuration
// PC--[USB]--KAW[S] <---[WiFi/UDP]--> KAW[C]

#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Ticker.h>
// FastLED for AtomS3 Lite onboard LED
#include <FastLED.h>

// Wireless communication method constants
#define ESPNOW 1
#define WIFI_UDP 2
#define WIFI_TCP 3

//-------------------------
// Wireless communication method selection: ESPNOW, WIFI_UDP, or WIFI_TCP
// #define WIRELESS WIFI_UDP
// #define WIRELESS WIFI_TCP
#define WIRELESS ESPNOW
//-------------------------

// Define symbols for conditional compilation
#if defined(WIRELESS) && WIRELESS == ESPNOW
#define WIRELESS_ESPNOW
#include <esp_now.h>
#elif defined(WIRELESS) && WIRELESS == WIFI_TCP
#define WIRELESS_TCP
#elif defined(WIRELESS) && WIRELESS == WIFI_UDP
#define WIRELESS_UDP
#endif

// Command definitions
#define CMD_START 0x01
#define CMD_STOP 0x02
#define CMD_STATUS 0x03

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

// Task to process received items
void recvTask(void *pvParameters) {
	RecvItem item;
	for (;;) {
		if (xQueueReceive(recvQueue, &item, portMAX_DELAY) == pdTRUE) {
			uint32_t currentTime = millis();
			if (item.len == 1) {
				uint8_t cmd = item.data[0];
				printf("[%d ms] Command received from ", currentTime);
				for (int i = 0; i < 6; i++) {
					printf("%02X", item.mac[i]);
					if (i < 5) printf(":");
				}
				printf(": ");
				switch (cmd) {
				default:
					printf("UNKNOWN (0x%02X)\n", cmd);
					break;
				}
				continue;
			}
			// データ受信時に内容とmillis()を表示
			//printf("[SERVER] Data received at %lu ms: %d bytes\n", (unsigned long)currentTime, item.len);
			//for (int i = 0; i < item.len; i++) {
			//	char c = item.data[i];
			//	if (c >= 32 && c <= 126) putchar(c); else putchar('.');
			//}
			//putchar('\n');
			// ...既存のスループット計算はそのまま...
			lastReceiveTime = currentTime;
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

#define PORT 12345
Ticker ticker;
#define SAMPLE_FREQ 250
// TDMA parameters
#define TDMA_FRAME_MS 24 // Frame period in milliseconds
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

#ifdef WIRELESS_ESPNOW
// ESP-NOW configuration
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast address
char packetBuffer[256];																						 // Buffer to accumulate packets
int packetCount = 0;																							 // Current number of packets in buffer
#elif defined WIRELESS_TCP
WiFiServer server(PORT);
WiFiClient client;
#elif defined WIRELESS_UDP
WiFiUDP udp;
#endif

const char *serverIP = "192.168.4.1"; // APのIP
char buf[1025];

uint32_t t0;
uint32_t lastReceiveTime = 0; // Track last receive time for SERVER
// Throughput measurement (server)
#define SERVER_THROUGHPUT_SAMPLE 100
uint16_t nThroughputSamples = 0;
uint32_t previousThroughputSamplingTime = 0;

// LED state tracking: true = transmitting(blue), false = stopped(green)
volatile uint8_t ledTransmittingState = 0;
// Track last sent command for reporting in send callback
volatile uint8_t lastCommandSent = 0;

// FastLED fallback
#define FALLBACK_NUM_LEDS 1
static CRGB fastled_leds[FALLBACK_NUM_LEDS];
static bool fastled_initialized = false;
static int fastled_pin = -1;

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

// Helper: update LED according to transmission state
static void updateTransmissionLED(bool transmitting)
{
	if (fastled_initialized)
	{
		if (transmitting)
			fastled_leds[0] = CRGB::Blue;
		else
			fastled_leds[0] = CRGB::Green;
		FastLED.show();
	}
}

volatile uint8_t fSample = 0;

void IRAM_ATTR onTicker()
{
	//    printf("fSample = %d", fSample);
	fSample = 1;
	//    printf("-> %d\n", fSample);
}

#ifdef WIRELESS_ESPNOW
// Callback function for when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
	if (status != ESP_NOW_SEND_SUCCESS)
	{
		printf("Last Packet Send Status = FAILED");
	}
//	else printf("Last Packet Send Status = SUCCESS");
	// Report which command this send corresponds to (if any)
	if (lastCommandSent == CMD_START)
	{
		printf(" (command: START)\n");
		lastCommandSent = 0; // Clear after reporting to avoid duplicate reports
	}
	else if (lastCommandSent == CMD_STOP)
	{
		printf(" (command: STOP)\n");
		lastCommandSent = 0; // Clear after reporting to avoid duplicate reports
	}
	else
	{
		// Silent for beacon and other data packets
	}
}

#endif

void setup()
{
	M5.begin();

	// Initialize FastLED fallback unconditionally (do not use M5.Led)
	initFastLEDFallback();
	// ensure initial color is green (stopped)
	fastled_leds[0] = CRGB(0, 30, 0);
	FastLED.show();

#ifdef WIRELESS_ESPNOW
	// Initialize ESP-NOW in STA mode (simpler, avoids AP overhead)
	WiFi.mode(WIFI_MODE_STA);
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
	// Start periodic beacon for TDMA (broadcast) - enqueue beacon to sendQueue from ISR
	beaconTicker.attach_ms(TDMA_FRAME_MS, [](){
		if (sendQueue == NULL) return;
		SendItem item;
		memcpy(item.addr, broadcastAddress, 6);
		item.data[0] = 0xBE; item.data[1] = 0xAC; item.len = 2;
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendFromISR(sendQueue, &item, &xHigherPriorityTaskWoken);
		if (xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
		//printf("[SERVER] Beacon sent at %lu ms\n", millis()); // debug: comment out for now
	});
#else
	// アクセスポイントモードに設定
	WiFi.softAP(ssid, password);
	printf("WiFi AP started, IP Address: %s\n", WiFi.softAPIP().toString());
#ifdef WIRELESS_TCP
	server.begin();
	printf("TCP server started on port %d\n", PORT);
#elif defined WIRELESS_UDP
	udp.begin(PORT);
	printf("Listening on UDP port %d\n", PORT);
#endif
	t0 = millis();
#endif
}

void loop()
{
	// Check for button press to send commands
	M5.update();
	static uint8_t lastButtonState = 0;
	static uint32_t lastButtonTime = 0;

	if (M5.BtnA.wasClicked())
	{
		lastButtonState = !lastButtonState;
		uint8_t cmd = lastButtonState ? CMD_START : CMD_STOP;
		printf("Sending command: %s\n", lastButtonState ? "START" : "STOP");
#ifdef WIRELESS_ESPNOW
		// Send command via ESP-NOW
		lastCommandSent = cmd;
		// Ensure peer exists for broadcast (some ESP-NOW setups require adding peer)
		if (!esp_now_is_peer_exist(broadcastAddress))
		{
			esp_now_peer_info_t peer = {};
			memcpy(peer.peer_addr, broadcastAddress, 6);
			peer.channel = 0;
			peer.encrypt = false;
			esp_err_t addres = esp_now_add_peer(&peer);
//			printf("esp_now_add_peer returned %d (%s)\n", addres, esp_err_to_name(addres));
		}
		// enqueue command to sendQueue for commTask
		if (sendQueue != NULL) {
			SendItem item;
			memcpy(item.addr, broadcastAddress, 6);
			item.data[0] = cmd;
			item.len = 1;
			if (xQueueSend(sendQueue, &item, pdMS_TO_TICKS(10)) != pdTRUE) {
				printf("sendQueue full - failed to enqueue cmd\n");
			}
		} else {
			// fallback: direct send
			esp_err_t espres = esp_now_send(broadcastAddress, &cmd, 1);
		}
#elif defined WIRELESS_TCP
			// Send command via TCP
			if (client && client.connected())
			{
				int written = client.write(&cmd, 1);
				printf("TCP: wrote %d bytes for command %s\n", written, lastButtonState ? "START" : "STOP");
			}
#elif defined WIRELESS_UDP
			// Send command via UDP
			udp.beginPacket(clientIP, clientPort);
			udp.write(&cmd, 1);
			int udpRes = udp.endPacket();
			printf("UDP: endPacket returned %d for command %s\n", udpRes, lastButtonState ? "START" : "STOP");
#endif
	}

	// Update server LED to reflect requested client state
	ledTransmittingState = lastButtonState;
	updateTransmissionLED(ledTransmittingState);

#ifdef WIRELESS_ESPNOW
	// ESP-NOW server receives data via callback (onDataRecv)
	// No polling needed, data is received asynchronously
	delay(100);

#elif defined WIRELESS_TCP
	// Accept incoming client connections
	if (!client)
	{
		client = server.available();
		if (client)
		{
			printf("[%d ms] Client connected\n", millis() - t0);
		}
	}

	// Read data from connected client
	if (client && client.available())
	{
		int len = client.available();
		if (len > 1024)
			len = 1024;
		int bytesRead = client.readBytes(buf, len);
		buf[bytesRead] = '\0';
		uint32_t currentTime = millis() - t0;
		uint32_t timeSinceLast = (lastReceiveTime == 0) ? 0 : (currentTime - lastReceiveTime);
		lastReceiveTime = currentTime;
		printf("[%d ms] TCP received %d bytes (%.1f ms since last): %s",
					 currentTime, bytesRead, (float)timeSinceLast, buf);
	}

#elif defined WIRELESS_UDP
	// Poll for incoming UDP packets
	int packetSize = udp.parsePacket();
	if (packetSize > 0)
	{
		// Read packet
		int len = packetSize;
		if (len > 1024)
			len = 1024;
		int bytesRead = udp.read(buf, len);
		if (bytesRead > 0)
		{
			buf[bytesRead] = '\0';
		}
		uint32_t currentTime = millis() - t0;
		uint32_t timeSinceLast = (lastReceiveTime == 0) ? 0 : (currentTime - lastReceiveTime);
		lastReceiveTime = currentTime;
		printf("[%d ms] UDP received %d bytes (%.1f ms since last) from %s:%d: %s",
					 currentTime, bytesRead, (float)timeSinceLast,
					 udp.remoteIP().toString().c_str(), udp.remotePort(), buf);
	}
#endif
}
