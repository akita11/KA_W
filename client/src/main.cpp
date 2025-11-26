// KA_W Client
#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Ticker.h>
#include <FastLED.h>

#define ESPNOW 1
#define WIFI_UDP 2
#define WIFI_TCP 3

#define WIRELESS ESPNOW
// #define WIRELESS UDP

#define CMD_START 0x01
#define CMD_STOP 0x02
#define CMD_STATUS 0x03

Ticker ticker;
#define SAMPLE_FREQ 250
#define FALLBACK_NUM_LEDS 1

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
	uint32_t currentTime = millis();
	if (len == 1)
	{
		uint8_t cmd = incomingData[0];
		printf("[%d ms] Command received from ", currentTime);
		for (int i = 0; i < 6; i++)
		{
			printf("%02X", mac[i]);
			if (i < 5)
				printf(":");
		}
		printf(": ");
		switch (cmd)
		{
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
		return;
	}
	printf("[%d ms] Received %d bytes from: ", currentTime, len);
	for (int i = 0; i < 6; i++)
	{
		printf("%02X", mac[i]);
		if (i < 5)
			printf(":");
	}
	printf(": ");
	for (int i = 0; i < len; i++)
		printf("%c", incomingData[i]);
	printf("\n");
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
#define DATA_PER_PACKET 3
// memo: ESPNOWは256バイトまで?

uint32_t lastReceiveTime, currentTime;

void loop()
{
	/*
	M5.update();
	if (M5.BtnA.wasPressed())
	{
		isTransmitting = !isTransmitting;
		printf("CLIENT: Transmission %s\n", isTransmitting ? "STARTED" : "STOPPED");
		updateTransmissionLED(isTransmitting);
	}
	if (!isTransmitting)
	{
		delay(10);
		return;
	}
	*/
	fSample = 0;
	while (fSample == 0)
		delayMicroseconds(10);
	if (isTransmitting == 1)
	{
		sprintf(buf, "%d,0.123,0.234,0.345,0.456,0.567,0.678\n", n);
		strcat(buf_packet, buf);
		n++;
		if (n == DATA_PER_PACKET)
		{
			n = 0;
			currentTime = millis();
			uint32_t timeSinceLast = (lastReceiveTime == 0) ? 0 : (currentTime - lastReceiveTime);
			lastReceiveTime = currentTime;
			printf("%d ms\n", timeSinceLast);
#ifdef WIRELESS_ESPNOW
			printf("Sending: %d bytes\n", strlen(buf_packet));
			if (esp_now_send(broadcastAddress, (uint8_t *)buf_packet, strlen(buf_packet) + 1) != ESP_OK)
				printf("Error sending data\n");
#elif defined(WIRELESS_TCP)
			if (client.connected())
			{
				client.print(buf);
				printf("Sent via TCP: %s", buf);
			}
			else
			{
				printf("TCP disconnected...reconnect\n");
				if (!client.connect(serverIP, PORT))
					printf("Reconnection failed\n");
			}
#elif defined(WIRELESS_UDP)
			printf("Sending via UDP to %s:%d: %s", serverIP, PORT, buf);
			udp.beginPacket(serverIP, PORT);
			udp.print(buf);
			if (!udp.endPacket())
				printf("Error sending UDP packet\n");
#endif
		}
		strcpy(buf_packet, "");
	}
}
