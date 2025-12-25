// Configuration
// PC--[USB]--KAW[S] <---[WiFi/UDP]--> KAW[C]

#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Ticker.h>
// FastLED for AtomS3 Lite onboard LED
#include <FastLED.h>

// Device type constants
#define SERVER 1
#define CLIENT 2

// Wireless communication method constants
#define ESPNOW 1
#define WIFI_UDP 2
#define WIFI_TCP 3

//-------------------------
// Wireless communication method selection: WIFI_TCP (only TCP used)
#define WIRELESS WIFI_TCP

// Device type selection: SERVER or CLIENT
#define TYPE SERVER
//#define TYPE CLIENT

#define CLIENT_ID 1 // client(sernsor)'s ID 
//-------------------------

// Define symbols for conditional compilation
#if defined(TYPE) && TYPE == SERVER
#define TYPE_SERVER
#elif defined(TYPE) && TYPE == CLIENT
#define TYPE_CLIENT
#endif

// Define symbols for conditional compilation
#if defined(WIRELESS) && WIRELESS == WIFI_TCP
#define WIRELESS_TCP
#elif defined(WIRELESS) && WIRELESS == WIFI_UDP
#define WIRELESS_UDP
#elif defined(WIRELESS) && WIRELESS == ESPNOW
#define WIRELESS_ESPNOW
#endif
#endif

// Buffering configuration
#define NUM_PACKETS 10 // Number of samples to buffer before sending

// Command definitions
#define CMD_START 0x01  // Start data transmission
#define CMD_STOP  0x02  // Stop data transmission
#define CMD_STATUS 0x03 // Request status

const char *ssid = "DualAccUDP_AP";
const char *password = "12345678";
#define PORT 12345
Ticker ticker;
#define SAMPLE_FREQ 250

WiFiServer server(PORT);
WiFiClient client;
#ifdef WIRELESS_UDP
WiFiUDP udp;
#endif

const char *serverIP = "192.168.4.1"; // APのIP
char buf[1025];

#ifdef TYPE_SERVER
uint32_t t0;
uint32_t lastReceiveTime = 0;  // Track last receive time for SERVER
// Throughput measurement (server)
uint32_t server_recv_window_start = 0;
uint32_t server_recv_window_bytes = 0;
#define SERVER_THROUGHPUT_WINDOW_MS 1000
#else
int n = 0;
volatile uint8_t isTransmitting = 1;  // Flag to control transmission (CLIENT only)
#endif

// LED state tracking: true = transmitting(blue), false = stopped(green)
volatile uint8_t ledTransmittingState = 0;
// Track last sent command for reporting in send callback
volatile uint8_t lastCommandSent = 0;

// FastLED fallback
#define FALLBACK_NUM_LEDS 1
static CRGB fastled_leds[FALLBACK_NUM_LEDS];
static bool fastled_initialized = false;
static int fastled_pin = -1;

static void initFastLEDFallback() {
    if (fastled_initialized) return;
    // Use ATOMS3 Lite LED pin 35
    fastled_pin = 35;
    FastLED.addLeds<WS2812B, 35, GRB>(fastled_leds, FALLBACK_NUM_LEDS);
    FastLED.setBrightness(64);
    fastled_initialized = true;
    fastled_leds[0] = CRGB::Green;
    FastLED.show();
}

// Helper: update LED according to transmission state
static void updateTransmissionLED(bool transmitting) {
    if (fastled_initialized) {
        if (transmitting) fastled_leds[0] = CRGB::Blue;
        else fastled_leds[0] = CRGB::Green;
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

// (ESP-NOW support removed) TCP/UDP-only logic below

void setup()
{
    M5.begin();

    // Initialize button notice for server
#ifdef TYPE_SERVER
    printf("SERVER: Press button to toggle CLIENT transmission\n");
#endif

    // Initialize FastLED fallback unconditionally (do not use M5.Led)
    initFastLEDFallback();
    // ensure initial color is green (stopped)
    fastled_leds[0] = CRGB(0, 30, 0);
    FastLED.show();

#ifdef TYPE_SERVER
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
#else // TYPE_CLIENT
    // Connect to WiFi network
    WiFi.begin(ssid, password);
    printf("Connecting to WiFi...");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        printf(".");
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        printf("\nConnected to WiFi, IP Address: %s\n", WiFi.localIP().toString());
    } else {
        printf("\nFailed to connect to WiFi\n");
    }

//    for (uint16_t i = 0; i < 1000; i++) buf[i] = '0' + (i % 10); buf[1000] = '\0';
    sprintf(buf, "0.123,0.234,0.345,0.456,0.567,0.678\n");

#ifdef WIRELESS_TCP
    printf("Connecting to TCP server at %s:%d...\n", serverIP, PORT);
    int maxAttempts = 5;
    while (!client.connect(serverIP, PORT) && maxAttempts-- > 0) {
        printf(".");
        delay(1000);
    }
    if (client.connected()) {
        printf("\nConnected to TCP server\n");
    } else {
        printf("\nFailed to connect to TCP server\n");
    }
#endif

    packetCount = 0;
    memset(packetBuffer, 0, sizeof(packetBuffer));
    ticker.attach_ms((int)(1000 / SAMPLE_FREQ), onTicker);
    // Set initial LED state on client according to isTransmitting
    ledTransmittingState = isTransmitting;
    updateTransmissionLED(isTransmitting);
#endif
}

void loop()
{
#ifdef TYPE_SERVER
#ifdef TYPE_SERVER

    // Check for button press to send commands
    M5.update();
    static uint8_t lastButtonState = 0;
    static uint32_t lastButtonTime = 0;
    
    if (M5.BtnA.wasPressed()) {
        // Prevent bouncing
        if (millis() - lastButtonTime > 500) {
            lastButtonState = !lastButtonState;
            uint8_t cmd = lastButtonState ? CMD_START : CMD_STOP;
            printf("[%d ms] Sending command: %s\n", millis() - t0, 
                   lastButtonState ? "START" : "STOP");
            
#if defined WIRELESS_TCP
            // Send command via TCP
            if (client && client.connected()) {
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
            lastButtonTime = millis();
        }
    }

    // Update server LED to reflect requested client state
    ledTransmittingState = lastButtonState;
    updateTransmissionLED(ledTransmittingState);

#if defined WIRELESS_TCP
    // Accept incoming client connections
    if (!client) {
        client = server.available();
        if (client) {
            printf("[%d ms] Client connected\n", millis() - t0);
        }
    } 
    
    // Read data from connected client
    if (client && client.available()) {
        int len = client.available();
        if (len > 1024) len = 1024;
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
    if (packetSize > 0) {
        // Read packet
        int len = packetSize;
        if (len > 1024) len = 1024;
        int bytesRead = udp.read(buf, len);
        if (bytesRead > 0) {
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

#else // TYPE_CLIENT
    // Check for button press (M5 buttonA to toggle transmission)
    M5.update();
    if (M5.BtnA.wasPressed()) {
        isTransmitting = !isTransmitting;
        printf("CLIENT: Transmission %s\n", isTransmitting ? "STARTED" : "STOPPED");
        // Update LED to reflect client transmission state
        ledTransmittingState = isTransmitting;
        updateTransmissionLED(isTransmitting);
    }

    // Skip sending if not transmitting
    if (!isTransmitting) {
        delay(10);
        return;
    }

    // Wait for the next sample tick
    fSample = 0;
    while(fSample == 0) delayMicroseconds(10);

#if defined WIRELESS_TCP
    // Send data over TCP if connected
    if (client.connected()) {
        client.print(buf);
        printf("Sent via TCP: %s", buf);
    } else {
        printf("TCP disconnected, attempting to reconnect to %s:%d\n", serverIP, PORT);
        if (!client.connect(serverIP, PORT)) {
            printf("Reconnection failed\n");
        }
    }

#elif defined WIRELESS_UDP
    // Send data over UDP
    printf("Sending via UDP to %s:%d: %s", serverIP, PORT, buf);
    udp.beginPacket(serverIP, PORT);
    udp.print(buf);
    if (!udp.endPacket()) {
        printf("Error sending UDP packet\n");
    }
#endif

#endif // TYPE_CLIENT
#endif // TYPE_SERVER
}
