// KA_W Server
// Configuration
// KA_W Server
// Configuration
// PC--[USB]--KAW[S] <---[TCP]--> KAW[C]

#define NUM_CLIENTS 2

#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <FastLED.h>

//#define DEBUG

// Debug-print macro: enable when DEBUG is defined
#ifndef DBG_PRINT
#ifdef DEBUG
#define DBG_PRINT(...) printf(__VA_ARGS__)
#else
#define DBG_PRINT(...) do{} while(0)
#endif
#endif

//#define USE_TCP

// Received data queue and task
typedef struct
{
	char clientID;
	uint8_t data[256];
	int len;
} RecvItem;

typedef struct
{
	char id;
	float yaw[24];
	float roll[24];
	float pitch[24];
	uint32_t tm;
	uint8_t fAlternatingError;
} DisplayItem;

static QueueHandle_t recvQueue = NULL;
static QueueHandle_t displayQueue = NULL;

// Track per-client connection (set when data received from client)
static bool client_connected[NUM_CLIENTS] = {0};

// Throughput sample count
#define SERVER_THROUGHPUT_SAMPLE 100

volatile uint8_t transferEnabled = 0; // STARTで1, STOPで0

volatile uint8_t fReceived = 0;
volatile uint32_t prevReceiveTime = 0;
volatile char prevClientID = 0;

// データ保持用配列: CLIENT x 12サンプル の yaw, roll, pitch
float client_yaw[NUM_CLIENTS][12];
float client_roll[NUM_CLIENTS][12];
float client_pitch[NUM_CLIENTS][12];
// Per-client payload circular buffer to hold up to 5 recent payloads (each payload = 12 samples)
#define PAYLOAD_BUFFER_DEPTH 5
typedef struct {
	float yaw[12];
	float roll[12];
	float pitch[12];
} PayloadBuf;
static PayloadBuf payload_buffer[NUM_CLIENTS][PAYLOAD_BUFFER_DEPTH];
static uint8_t payload_head[NUM_CLIENTS] = {0}; // next write index
static uint8_t payload_count[NUM_CLIENTS] = {0}; // number of stored payloads (<=DEPTH)
volatile uint32_t lastDisplayTime = 0;

// FastLED fallback
#define FALLBACK_NUM_LEDS 1
static CRGB fastled_leds[FALLBACK_NUM_LEDS];
static bool fastled_initialized = false;
static int fastled_pin = -1;

// UDP
WiFiUDP udp;
const uint16_t UDP_PORT = 12345;

#define START0 0xAA
#define START1 0x01
#define STOP0 0xBB
#define STOP1 0x00

// forward declaration
static void printAndResetIntervalAverages();

#if defined(USE_TCP)
WiFiServer tcpServer(UDP_PORT);
#endif

uint8_t nShowDebug = 0;
uint32_t sampleSeq = 0;
uint32_t t0 = 0;
uint8_t fAlternatingReceiveError = 0;
uint32_t lastReceiveTime = 0; // Track last receive time for SERVER
uint16_t nThroughputSamples = 0;
uint32_t previousThroughputSamplingTime = 0;
// per-client last receive timestamp (ms)
static uint32_t last_recv_ms[NUM_CLIENTS] = {0};
// per-client interval accumulators (sum of intervals in ms and count)
static uint64_t sum_intervals_ms[NUM_CLIENTS] = {0};
static uint32_t count_intervals[NUM_CLIENTS] = {0};

void processReceivedData(RecvItem item)
{
	uint32_t currentTime = millis();
	// per-client elapsed will be computed below
	// 受信元クライアントIDを先に表示してからペイロード内容を表示
	char srcId = (item.len > 0) ? (char)item.data[0] : '?';
	// print only elapsed ms per-client (computed after determining client index)
	// placeholder message; exact elapsed printed below when client index known
/*
		for (int i = 0; i < item.len; i++)
		{
			char c = item.data[i];
			if (c >= 32 && c <= 126)
				putchar(c);
			else
				putchar('.');
		}
		putchar('\n');
*/

	char id = (item.len > 0) ? item.data[0] : '?';
	// mark client as seen/connected and compute per-client elapsed
	if (id >= '1' && id <= '0' + NUM_CLIENTS)
	{
		int cidx = id - '1';
		if (!client_connected[cidx])
		{
			client_connected[cidx] = true;
			DBG_PRINT("[SERVER] CLIENT %c marked connected\n", id);
			// If all clients are now connected, set LED to green immediately
			bool allConn = true;
			for (int _i = 0; _i < NUM_CLIENTS; ++_i) if (!client_connected[_i]) { allConn = false; break; }
			if (allConn)
			{
				fastled_leds[0] = CRGB(0, 30, 0);
				FastLED.show();
			}
		}
		// compute per-client elapsed ms since previous receive
		uint32_t elapsed_client_ms = (last_recv_ms[cidx] == 0) ? 0 : (currentTime - last_recv_ms[cidx]);
		//printf("[SERVER] CLIENT %c elapsed %lu ms since previous receive, %d bytes\n", id, (unsigned long)elapsed_client_ms, item.len);
		// accumulate interval only when transfer is active (between START and STOP)
		if (transferEnabled && last_recv_ms[cidx] != 0)
		{
			sum_intervals_ms[cidx] += (uint64_t)elapsed_client_ms;
			count_intervals[cidx]++;
		}
		last_recv_ms[cidx] = currentTime;
	}

	// データパースと格納
	for (uint8_t i = 0; i < 12; i++)
	{
		char sample[19];
		memcpy(sample, &item.data[1 + i * 18], 18);
		sample[18] = '\0';
		char yaw_str[7], roll_str[7], pitch_str[7];
		memcpy(yaw_str, &sample[0], 6);
		yaw_str[6] = '\0';
		memcpy(roll_str, &sample[6], 6);
		roll_str[6] = '\0';
		memcpy(pitch_str, &sample[12], 6);
		pitch_str[6] = '\0';
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
		// also store into circular payload buffer
		payload_buffer[client_idx][payload_head[client_idx]].yaw[i] = yaw;
		payload_buffer[client_idx][payload_head[client_idx]].roll[i] = roll;
		payload_buffer[client_idx][payload_head[client_idx]].pitch[i] = pitch;
	}
	// advance circular head and count for this client's payload buffer
	if (id >= '1' && id <= '0' + NUM_CLIENTS)
	{
		int cidx = id - '1';
		payload_head[cidx] = (payload_head[cidx] + 1) % PAYLOAD_BUFFER_DEPTH;
		if (payload_count[cidx] < PAYLOAD_BUFFER_DEPTH) payload_count[cidx]++;
	}
	// (displaying combined payloads is handled below using per-client block buffers)
	lastReceiveTime = currentTime;
	prevReceiveTime = currentTime;
	fReceived = (fReceived + 1) % 10;

	// If this is CLIENT '2' and transfer is enabled, prepare a combined display item
	if (id == '2' && transferEnabled)
	{
		DisplayItem dispItem;
		dispItem.id = id;
		dispItem.tm = 0;
		dispItem.fAlternatingError = fAlternatingReceiveError;
		// Determine latest payload index for each client
		for (int c = 0; c < NUM_CLIENTS; ++c)
		{
			if (payload_count[c] == 0)
			{
				// no data for this client yet: fill zeros
				for (int i = 0; i < 12; ++i)
				{
					dispItem.yaw[c * 12 + i] = 0.0f;
					dispItem.roll[c * 12 + i] = 0.0f;
					dispItem.pitch[c * 12 + i] = 0.0f;
				}
			}
			else
			{
				int last_idx = (payload_head[c] + PAYLOAD_BUFFER_DEPTH - 1) % PAYLOAD_BUFFER_DEPTH;
				for (int i = 0; i < 12; ++i)
				{
					dispItem.yaw[c * 12 + i] = payload_buffer[c][last_idx].yaw[i];
					dispItem.roll[c * 12 + i] = payload_buffer[c][last_idx].roll[i];
					dispItem.pitch[c * 12 + i] = payload_buffer[c][last_idx].pitch[i];
				}
			}
		}
		// compute tm using micros() delta as before
		uint32_t t1 = micros();
		dispItem.tm = (t0 == 0) ? 0 : (t1 - t0);
		t0 = t1;
		// enqueue for display
		xQueueSend(displayQueue, &dispItem, 0);
		lastDisplayTime = currentTime;
	}

	// スループット計算（オプション）
	// prevReceiveTime = currentTime;
	// nThroughputSamples++;
	// if (nThroughputSamples == SERVER_THROUGHPUT_SAMPLE) { ... }
}

// Print per-client average receive interval (ms) for the last START..STOP session and reset accumulators
static void printAndResetIntervalAverages()
{
	DBG_PRINT("[SERVER] Per-client average receive interval (ms) for last session:\n");
	for (int i = 0; i < NUM_CLIENTS; ++i)
	{
		printf("#");
		if (count_intervals[i] == 0)
		{
			printf("%d,-,-,", i + 1);
		}
		else
		{
			uint32_t avg = (uint32_t)(sum_intervals_ms[i] / count_intervals[i]);
			printf("%d,%lu,%u,", i + 1, (unsigned long)avg, count_intervals[i]);
		}
		// reset accumulators and last receive time for next session
		sum_intervals_ms[i] = 0;
		count_intervals[i] = 0;
		last_recv_ms[i] = 0;
	}
	printf("\n");
}

void recvTask(void *pvParameters)
{
	RecvItem item;
	for (;;)
	{
		// Support both UDP and TCP server modes
#if defined(USE_TCP)
		WiFiClient c = tcpServer.available();
		if (c)
		{
			int len = c.read((uint8_t *)item.data, sizeof(item.data));
			if (len > 0)
			{
				item.len = len;
				// If this is a STOP control (maybe received via UDP/TCP loopback), print averages and reset
				if (item.len == 2 && (uint8_t)item.data[0] == STOP0 && (uint8_t)item.data[1] == STOP1)
				{
					transferEnabled = 0;
					printAndResetIntervalAverages();
				}
				processReceivedData(item);
			}
		}
#else
		int packetSize = udp.parsePacket();
		if (packetSize > 0)
		{
			int len = udp.read((char *)item.data, sizeof(item.data));
			if (len > 0)
			{
				item.len = len;
				// If this is a STOP control (server may receive its own broadcast), print averages and reset
				if (item.len == 2 && (uint8_t)item.data[0] == STOP0 && (uint8_t)item.data[1] == STOP1)
				{
					transferEnabled = 0;
					printAndResetIntervalAverages();
				}
				processReceivedData(item);
			}
		}
#endif
		vTaskDelay(pdMS_TO_TICKS(10)); // Small delay
	}
}

#define SAMPLE_FREQ 250

// Communication task and queue
typedef struct
{
	uint8_t data[32];
	size_t len;
} SendItem;

static QueueHandle_t sendQueue = NULL;
static TaskHandle_t commTaskHandle = NULL;

void commTask(void *pvParameters)
{
	SendItem item;
	vTaskDelay(pdMS_TO_TICKS(500)); // Wait for WiFi/TCP to fully initialize
	for (;;)
	{
		if (xQueueReceive(sendQueue, &item, portMAX_DELAY) == pdTRUE)
		{
			vTaskDelay(pdMS_TO_TICKS(2)); // Small delay before send
#if defined(USE_TCP)
			// In TCP mode, send the queued payload to each client individually
			for (int c = 0; c < NUM_CLIENTS; ++c)
			{
				IPAddress clientIP(192, 168, 4, c + 2);
				WiFiClient tcp;
				if (tcp.connect(clientIP, UDP_PORT))
				{
					tcp.write(item.data, item.len);
					tcp.stop();
				}
				else
				{
					DBG_PRINT("[SERVER] commTask: TCP connect failed to %s\n", clientIP.toString().c_str());
				}
				vTaskDelay(pdMS_TO_TICKS(2));
			}
#else
			// Default behavior: send queued item as UDP broadcast
			IPAddress bc(192, 168, 4, 255);
			udp.beginPacket(bc, UDP_PORT);
			udp.write(item.data, item.len);
			udp.endPacket();
#endif
		}
	}
}

// Send control (START/STOP) to a specific client IP (repeat logic executed by caller)
static void sendControlToClient(int client_idx, bool isStart)
{
	IPAddress clientIP(192, 168, 4, client_idx + 2); // client_idx 0 -> .2
	uint8_t msg[2];
	msg[0] = isStart ? START0 : STOP0;
	msg[1] = isStart ? START1 : STOP1;
#if defined(USE_TCP)
	WiFiClient tcp;
	if (tcp.connect(clientIP, UDP_PORT))
	{
		tcp.write(msg, 2);
		tcp.stop();
		// Mark this client as connected when TCP connect succeeds
		if (client_idx >= 0 && client_idx < NUM_CLIENTS)
		{
				if (!client_connected[client_idx])
				{
					client_connected[client_idx] = true;
					DBG_PRINT("[SERVER] CLIENT %d marked connected (via TCP connect)\n", client_idx + 1);
				// If all clients are now connected, set LED to green immediately
				bool allConn = true;
				for (int _i = 0; _i < NUM_CLIENTS; ++_i) if (!client_connected[_i]) { allConn = false; break; }
				if (allConn)
				{
					fastled_leds[0] = CRGB(0, 30, 0);
					FastLED.show();
				}
			}
		}
	}
	else
	{
		DBG_PRINT("[SERVER] TCP connect failed to %s\n", clientIP.toString().c_str());
	}
#else
	udp.beginPacket(clientIP, UDP_PORT);
	udp.write(msg, 2);
	udp.endPacket();
#endif
}

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

// beacon ticker and periodic beaconing removed; server uses START/STOP control

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
	DBG_PRINT("AP IP address: %s\n", IP.toString().c_str());

	// Wait for all clients to connect to the AP before proceeding
	// Indicate waiting with orange LED
	fastled_leds[0] = CRGB(30, 15, 0); // orange
	FastLED.show();
	while (WiFi.softAPgetStationNum() < NUM_CLIENTS)
	{
		// brief delay while waiting
		delay(200);
	}
	// all clients are connected -> set green
	fastled_leds[0] = CRGB(0, 30, 0);
	FastLED.show();

	// Start UDP listener
	// Start network listener depending on transport
#if defined(USE_TCP)
		tcpServer.begin();
		DBG_PRINT("TCP server started on port %d\n", UDP_PORT);
#else
		if (!udp.begin(UDP_PORT))
		{
			DBG_PRINT("Failed to start UDP on port %d\n", UDP_PORT);
		}
		else
		{
			DBG_PRINT("UDP server started on port %d\n", UDP_PORT);
		}
#endif
	vTaskDelay(pdMS_TO_TICKS(100));

	// create send queue and communication task (pinned to core 0)
	sendQueue = xQueueCreate(10, sizeof(SendItem));
	if (sendQueue == NULL)
	{
		DBG_PRINT("Failed to create sendQueue\n");
	}
	else
	{
		xTaskCreatePinnedToCore(commTask, "CommTask", 4096, NULL, configMAX_PRIORITIES - 2, &commTaskHandle, 0);
	}

	// create receive queue and task
	recvQueue = xQueueCreate(16, sizeof(RecvItem));
	if (recvQueue == NULL)
	{
		DBG_PRINT("Failed to create recvQueue\n");
	}
	else
	{
		xTaskCreatePinnedToCore(recvTask, "RecvTask", 4096, NULL, configMAX_PRIORITIES - 3, NULL, 1);
	}

	// create display queue
	displayQueue = xQueueCreate(4, sizeof(DisplayItem));
	if (displayQueue == NULL)
	{
		DBG_PRINT("Failed to create displayQueue\n");
	}
}

void loop()
{
	M5.update();

	// Update LED according to whether all clients are connected and transfer state
	bool allConnected = true;
	for (int i = 0; i < NUM_CLIENTS; ++i)
		if (!client_connected[i]) { allConnected = false; break; }
	if (!allConnected)
	{
		// orange while waiting for all clients
		fastled_leds[0] = CRGB(30, 15, 0);
	}
	else
	{
		// when all connected: green if stopped, yellow if transferring
		if (transferEnabled)
			fastled_leds[0] = CRGB(30, 30, 0);
		else
			fastled_leds[0] = CRGB(0, 30, 0);
	}
	FastLED.show();

	if (M5.BtnA.wasClicked())
	{
		// Toggle transfer state and immediately notify clients via UDP
		transferEnabled = !transferEnabled;
		if (transferEnabled)
		{
			DBG_PRINT("Transfer ENABLED\n");
			sampleSeq = 0;
			// Send START to each client individually (with redundancy)
			for (int c = 0; c < NUM_CLIENTS; ++c)
			{
				for (int rep = 0; rep < 3; ++rep)
				{
					sendControlToClient(c, true);
					vTaskDelay(pdMS_TO_TICKS(10));
				}
			}
		}
		else
		{
			DBG_PRINT("Transfer DISABLED\n");
			// Stop display queue and notify each client to stop
			xQueueReset(displayQueue);
			for (int c = 0; c < NUM_CLIENTS; ++c)
			{
				for (int rep = 0; rep < 3; ++rep)
				{
					sendControlToClient(c, false);
					vTaskDelay(pdMS_TO_TICKS(10));
				}
			}
			// Print average intervals for each client for the session that just ended
			printAndResetIntervalAverages();
		}
		FastLED.show();
	}

	DisplayItem item;
	if (xQueueReceive(displayQueue, &item, 0) == pdTRUE)
	{
		if (transferEnabled)
		{
			for (int i = 0; i < 12; i++)
			{
				if (!transferEnabled)
					break;
				printf(",%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", item.tm / 12, item.roll[i], item.pitch[i], item.yaw[i], item.roll[12 + i], item.pitch[12 + i], item.yaw[12 + i]);
			}
		}
	}

    delay(10);
}
