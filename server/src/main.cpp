// KA_W Server
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
	float q0[24];
	float q1[24];
	float q2[24];
	float q3[24];
	uint32_t tm;
} DisplayItem;

static QueueHandle_t recvQueue = NULL;
static QueueHandle_t displayQueue = NULL;

// Track per-client connection (set when data received from client)
static bool client_connected[NUM_CLIENTS] = {0};

volatile uint8_t transferEnabled = 0; // STARTで1, STOPで0

// Per-client payload circular buffer to hold up to 5 recent payloads (each payload = 12 samples)
#define PAYLOAD_BUFFER_DEPTH 5
typedef struct {
	float q0[12];
	float q1[12];
	float q2[12];
	float q3[12];
} PayloadBuf;
static PayloadBuf payload_buffer[NUM_CLIENTS][PAYLOAD_BUFFER_DEPTH];
static uint8_t payload_head[NUM_CLIENTS] = {0}; // next write index
static uint8_t payload_count[NUM_CLIENTS] = {0}; // number of stored payloads (<=DEPTH)

// FastLED fallback
#define FALLBACK_NUM_LEDS 1
static CRGB fastled_leds[FALLBACK_NUM_LEDS];
static bool fastled_initialized = false;

// UDP
WiFiUDP udp;
const uint16_t UDP_PORT = 12345;

#define START0 0xAA
#define START1 0x01
#define STOP0 0xBB
#define STOP1 0x00

// forward declaration
static void printAndResetIntervalAverages();
static bool allClientsConnected();

#if defined(USE_TCP)
WiFiServer tcpServer(UDP_PORT);
#endif

uint32_t sampleSeq = 0;
uint32_t t0 = 0;
// per-client last receive timestamp (ms)
static uint32_t last_recv_ms[NUM_CLIENTS] = {0};
// per-client interval accumulators (sum of intervals in ms and count)
static uint64_t sum_intervals_ms[NUM_CLIENTS] = {0};
static uint32_t count_intervals[NUM_CLIENTS] = {0};

static bool allClientsConnected()
{
	for (int i = 0; i < NUM_CLIENTS; ++i)
		if (!client_connected[i]) return false;
	return true;
}

void processReceivedData(RecvItem item)
{
	uint32_t currentTime = millis();

	char id = (item.len > 0) ? item.data[0] : '?';
	// mark client as seen/connected and compute per-client elapsed
	if (id >= '1' && id <= '0' + NUM_CLIENTS)
	{
		int cidx = id - '1';
		if (!client_connected[cidx])
		{
			client_connected[cidx] = true;
			DBG_PRINT("[SERVER] CLIENT %c marked connected\n", id);
			if (allClientsConnected())
			{
				fastled_leds[0] = CRGB(0, 30, 0);
				FastLED.show();
			}
		}
		// compute per-client elapsed ms since previous receive
		uint32_t elapsed_client_ms = (last_recv_ms[cidx] == 0) ? 0 : (currentTime - last_recv_ms[cidx]);
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
		char sample[21];
		memcpy(sample, &item.data[1 + i * 20], 20);
		sample[20] = '\0';
		char q0_str[8], q1_str[8], q2_str[8], q3_str[8];
		memcpy(q0_str, &sample[0], 5);
		q0_str[5] = '\0';
		memcpy(q1_str, &sample[5], 5);
		q1_str[5] = '\0';
		memcpy(q2_str, &sample[10], 5);
		q2_str[5] = '\0';
		memcpy(q3_str, &sample[15], 5);
		q3_str[5] = '\0';
		int iq0 = atoi(q0_str);
		int iq1 = atoi(q1_str);
		int iq2 = atoi(q2_str);
		int iq3 = atoi(q3_str);
		float q0 = iq0 / 1000.0f;
		float q1 = iq1 / 1000.0f;
		float q2 = iq2 / 1000.0f;
		float q3 = iq3 / 1000.0f;
		int client_idx = id - '1'; // '1' -> 0, '2' -> 1
		// store into circular payload buffer
		payload_buffer[client_idx][payload_head[client_idx]].q0[i] = q0;
		payload_buffer[client_idx][payload_head[client_idx]].q1[i] = q1;
		payload_buffer[client_idx][payload_head[client_idx]].q2[i] = q2;
		payload_buffer[client_idx][payload_head[client_idx]].q3[i] = q3;
	}
	// advance circular head and count for this client's payload buffer
	if (id >= '1' && id <= '0' + NUM_CLIENTS)
	{
		int cidx = id - '1';
		payload_head[cidx] = (payload_head[cidx] + 1) % PAYLOAD_BUFFER_DEPTH;
		if (payload_count[cidx] < PAYLOAD_BUFFER_DEPTH) payload_count[cidx]++;
	}

	// If this is CLIENT '2' and transfer is enabled, prepare a combined display item
	if (id == '2' && transferEnabled)
	{
		DisplayItem dispItem;
		dispItem.id = id;
		dispItem.tm = 0;
		// Determine latest payload index for each client
		for (int c = 0; c < NUM_CLIENTS; ++c)
		{
			if (payload_count[c] == 0)
			{
				// no data for this client yet: fill zeros
				for (int i = 0; i < 12; ++i)
				{
					dispItem.q0[c * 12 + i] = 0.0f;
					dispItem.q1[c * 12 + i] = 0.0f;
					dispItem.q2[c * 12 + i] = 0.0f;
					dispItem.q3[c * 12 + i] = 0.0f;
				}
			}
			else
			{
				int last_idx = (payload_head[c] + PAYLOAD_BUFFER_DEPTH - 1) % PAYLOAD_BUFFER_DEPTH;
				for (int i = 0; i < 12; ++i)
				{
					dispItem.q0[c * 12 + i] = payload_buffer[c][last_idx].q0[i];
					dispItem.q1[c * 12 + i] = payload_buffer[c][last_idx].q1[i];
					dispItem.q2[c * 12 + i] = payload_buffer[c][last_idx].q2[i];
					dispItem.q3[c * 12 + i] = payload_buffer[c][last_idx].q3[i];
				}
			}
		}
		// compute tm using micros() delta
		uint32_t t1 = micros();
		dispItem.tm = (t0 == 0) ? 0 : (t1 - t0);
		t0 = t1;
		// enqueue for display
		xQueueSend(displayQueue, &dispItem, 0);
	}
}

// Print per-client average receive interval (ms) for the last START..STOP session and reset accumulators
static void printAndResetIntervalAverages()
{
	DBG_PRINT("[SERVER] Per-client average receive interval (ms) for last session:\n");
	printf("#");
	for (int i = 0; i < NUM_CLIENTS; ++i)
	{
		if (count_intervals[i] == 0)
		{
			printf("%d,0,-,", i + 1);
		}
		else
		{
			double avg = (double)sum_intervals_ms[i] / (double)count_intervals[i];
			printf("%d,%.2f,%u,", i + 1, avg, count_intervals[i]);
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
#if defined(USE_TCP)
		WiFiClient c = tcpServer.available();
		if (c)
		{
			int len = c.read((uint8_t *)item.data, sizeof(item.data));
			if (len > 0)
			{
				item.len = len;
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
				if (item.len == 2 && (uint8_t)item.data[0] == STOP0 && (uint8_t)item.data[1] == STOP1)
				{
					transferEnabled = 0;
					printAndResetIntervalAverages();
				}
				processReceivedData(item);
			}
		}
#endif
		vTaskDelay(pdMS_TO_TICKS(10));
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
			vTaskDelay(pdMS_TO_TICKS(2));
#if defined(USE_TCP)
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
			IPAddress bc(192, 168, 4, 255);
			udp.beginPacket(bc, UDP_PORT);
			udp.write(item.data, item.len);
			udp.endPacket();
#endif
		}
	}
}

// Send control (START/STOP) to a specific client IP
static void sendControlToClient(int client_idx, bool isStart)
{
	IPAddress clientIP(192, 168, 4, client_idx + 2);
	uint8_t msg[2];
	msg[0] = isStart ? START0 : STOP0;
	msg[1] = isStart ? START1 : STOP1;
#if defined(USE_TCP)
	WiFiClient tcp;
	if (tcp.connect(clientIP, UDP_PORT))
	{
		tcp.write(msg, 2);
		tcp.stop();
		if (client_idx >= 0 && client_idx < NUM_CLIENTS)
		{
			if (!client_connected[client_idx])
			{
				client_connected[client_idx] = true;
				DBG_PRINT("[SERVER] CLIENT %d marked connected (via TCP connect)\n", client_idx + 1);
				if (allClientsConnected())
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
	FastLED.addLeds<WS2812B, 35, GRB>(fastled_leds, FALLBACK_NUM_LEDS);
	FastLED.setBrightness(64);
	fastled_initialized = true;
	fastled_leds[0] = CRGB::Green;
	FastLED.show();
}

void setup()
{
	M5.begin();
	Serial.setTxBufferSize(2048);

	initFastLEDFallback();
	fastled_leds[0] = CRGB(0, 30, 0);
	FastLED.show();

	// Initialize WiFi in AP mode
	WiFi.mode(WIFI_AP);
	WiFi.softAP("KAW_Server", "password123");
	IPAddress IP = WiFi.softAPIP();
	DBG_PRINT("AP IP address: %s\n", IP.toString().c_str());

	// Wait for all clients to connect to the AP before proceeding
	fastled_leds[0] = CRGB(30, 15, 0); // orange
	FastLED.show();
	while (WiFi.softAPgetStationNum() < NUM_CLIENTS)
	{
		delay(200);
	}
	fastled_leds[0] = CRGB(0, 30, 0);
	FastLED.show();

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

	// Update LED according to connection and transfer state
	if (!allClientsConnected())
	{
		fastled_leds[0] = CRGB(30, 15, 0); // orange while waiting
	}
	else
	{
		if (transferEnabled)
			fastled_leds[0] = CRGB(30, 30, 0); // yellow if transferring
		else
			fastled_leds[0] = CRGB(0, 30, 0); // green if stopped
	}
	FastLED.show();

	if (M5.BtnA.wasClicked())
	{
		transferEnabled = !transferEnabled;
		if (transferEnabled)
		{
			DBG_PRINT("Transfer ENABLED\n");
			sampleSeq = 0;
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
			xQueueReset(displayQueue);
			for (int c = 0; c < NUM_CLIENTS; ++c)
			{
				for (int rep = 0; rep < 3; ++rep)
				{
					sendControlToClient(c, false);
					vTaskDelay(pdMS_TO_TICKS(10));
				}
			}
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
				printf(",%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", item.tm / 12, item.q0[i], item.q1[i], item.q2[i], item.q3[i], item.q0[12 + i], item.q1[12 + i], item.q2[12 + i], item.q3[12 + i]);
			}
		}
	}

	delay(10);
}
