// KA_W Client
#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Ticker.h>
#include <FastLED.h>

#define CLIENT_ID 1 // or 2, but for beaconChecker, maybe not needed
#define NUM_CLIENTS 2

Ticker ticker;
#define SAMPLE_FREQ 250
#define FALLBACK_NUM_LEDS 1

// TDMA not used in beaconChecker; timing handled elsewhere if needed

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
	uint8_t data[256];
	int len;
} RecvItem;

static QueueHandle_t recvQueue = NULL;

// TCP client
WiFiClient tcpClient;

// Task to process received items (client)
void recvTask(void *pvParameters) {
	for(;;) {
		if (tcpClient.available()) {
			uint8_t data[256];
			int len = tcpClient.readBytes((char*)data, sizeof(data));
			if (len == 2 && data[0] == 0xBE && data[1] == 0xAC) {
				uint32_t currentTime = millis();
				uint32_t interval = (last_beacon_ms == 0) ? 0 : (currentTime - last_beacon_ms);
				//printf("Beacon interval: %d ms / 48ms), Seq: %d", interval, last_beacon_seq + 1);
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
			}
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

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

void setup()
{
	auto cfg = M5.config();
	cfg.external_imu = false;
	cfg.internal_imu = false;
	cfg.internal_spk = false;
	cfg.internal_mic = false;

	M5.begin(cfg);

	initFastLEDFallback();
	fastled_leds[0] = CRGB(0, 30, 0);
	FastLED.show();

	// Connect to WiFi AP
	WiFi.begin("KAW_Server", "password123");
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
	}
	printf("Connected to WiFi\n");

	// Connect to TCP server
	if (tcpClient.connect("192.168.4.1", 12345)) { // Server IP, assuming default AP IP
		printf("Connected to TCP server\n");
	} else {
		printf("Failed to connect to TCP server\n");
	}
	// create receive task
	xTaskCreatePinnedToCore(recvTask, "RecvTask", 4096, NULL, configMAX_PRIORITIES-3, NULL, 0);

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
	if (beacon_lost == true){
		fastled_leds[0] = CRGB(30, 0, 0);
	}
	else{
		if ((last_beacon_seq % BEACON_CHECK_CYCLE) < BEACON_CHECK_CYCLE / 2)
		{
			fastled_leds[0] = CRGB(0, 30, 0);
		}
		else
		{
			fastled_leds[0] = CRGB(0, 0, 30);
		}
	}
	FastLED.show();
	delay(4);
}
