/*
 * ==============================================================
 *  Project:  EoRa-S3-900TB-with-FreeRTOS
 *  File:     EoRa_S3_900TB_FreeRTOS_AutoDutyCycle_Transmitter.ino
 * ===============================================================
 *
 *  Developer and Founder:   William Lucid, AB9NQ (Tech500)
 *  Hardware: Ebyte EoRa-S3-900TB (SX1262, ESP32-S3)
 *  Library:  RadioLib v7.5.0 by Jan Gromes
 *
 *  Description:
 *    LoRa WOR (Wake-On-Radio) duty cycle based camera power
 *    control system. TX sends a wake packet followed by a
 *    command packet. RX wakes from deep sleep on WOR interrupt,
 *    receives the command, controls camera power via GPIO,
 *    and sends an ACK back to TX.
 *
 *  Features:
 *    - WOR duty cycle deep sleep for low power operation
 *    - Struct-based messaging with type identification
 *      (0xAA = wake, 0xBB = command, 0xFF = ACK)
 *    - Interrupt-driven ACK handshake via radio.setDio1Action()
 *    - NTP timestamp delivery with Indianapolis DST support
 *    - AsyncWebServer trigger for remote camera ON/OFF control
 *    - EXT0 Timer Wake of 120 seconds for auto-off countdown
 *
 *  Development Assistance:
 *    Claude AI (Anthropic - claude.ai)
 *    Significant debugging and design assistance provided by
 *    Claude AI throughout development, including:
 *      - Interrupt handling via radio.setDio1Action()
 *      - WOR timing and deep sleep synchronization
 *      - Struct-based packet type identification
 *      - ACK handshake state machine design
 *      - ISR safety and watchdog crash resolution
 *     Additional AI contributions:
 *     ChatGPT (Open AI). Copilot (Microfoft); and Gemini (Google).
 *  References:
 *    RadioLib Examples: https://github.com/jgromes/RadioLib
 *    Ebyte EoRa-S3-900TB Examples:
 *    https://github.com/Tech500/Ebyte-EoRa-S3-900TB-RadioLib-Examples
 *
 *  Date:    5 March 2026
 * ============================================================
 */

#define EoRa_PI_V1
#include <Arduino.h>
#include "RadioLib.h"
#include "boards.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include "radio_eora.h"     // your SX1262 wrapper
#include "packet_struct.h"  // your LoraPacket struct

#include "index7.h"         // HTML7 + processor7


extern SX1262 radio;

// -------------------- WiFi Credentials --------------------
const char* ssid = "yourSSID";
const char* password = "yourPassword";

// ---------------------------
// Timestamp helper
// ---------------------------
uint32_t getTimestampSafe() {
  time_t now;
  if (time(&now)) return (uint32_t)now;
  return (uint32_t)(millis() / 1000);
}

bool initNTP() {
  configTzTime("EST5EDT,M3.2.0/2,M11.1.0/2",
               "pool.ntp.org",
               "time.nist.gov");

  struct tm info;
  unsigned long start = millis();

  while (!getLocalTime(&info)) {
    if (millis() - start > 5000) return false;
    delay(200);
  }
  return true;
}

// ---------------------------
// Local timestamp (Indianapolis, DST auto)
// ---------------------------
String getLocalTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "NTP not ready";
  }

  char buf[49];
  strftime(buf, sizeof(buf), "%A %Y-%m-%d %I:%M:%S %p", &timeinfo);
  return String(buf);
}

// -------------------- Web Server --------------------
AsyncWebServer server(80);
String linkAddress = "192.168.12.27:80";

// -------------------- TX Flags --------------------
volatile bool sendRequested = false;
volatile bool ackReceived = false;
volatile uint8_t command = 1;

void IRAM_ATTR onDio1() {
    ackReceived = true;
}

// -------------------- Forward Declarations --------------------
void txTask(void* parameter);
void ntpTask(void* parameter);
//bool sendWorCommand(command);
bool waitForAck(uint32_t timeoutMs);
void wifi_Start();

LoraPacket pkt;

String processor7(const String& var);

bool waitForAck(uint32_t timeoutMs = 3000) {
    radio.setDio1Action(onDio1);
    ackReceived = false;
    radio.startReceive();

    uint32_t start = millis();

    while (millis() - start < timeoutMs) {
        if (ackReceived) {
            ackReceived = false;

            AckPacket ack;
            int st = radio.readData((uint8_t*)&ack, sizeof(ack));

            if (st == RADIOLIB_ERR_NONE && ack.type == 0xFF) {
                Serial.println("TX: ACK received");
                radio.standby();  // ← ADD HERE (success path)
                return true;
            }

            radio.startReceive();
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    Serial.println("TX: No ACK");
    radio.standby();  // ← ADD HERE (timeout path)
    return false;
}

void waitForNTP() {
    struct tm timeinfo;
    Serial.print("[NTP] Waiting for sync");
    while (!getLocalTime(&timeinfo)) {
        Serial.print(".");
        delay(500);
    }
    Serial.println(" synced!");
}

void syncNTP() {
    Serial.print("[NTP] Syncing time");
    configTzTime("EST5EDT,M3.2.0,M11.1.0", "pool.ntp.org");
    
    struct tm timeinfo;
    int retries = 0;
    while (!getLocalTime(&timeinfo) && retries < 20) {
        Serial.print(".");
        delay(500);
        retries++;
    }

    if (retries < 20) {
        char buf[32];
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
        Serial.printf("\n[NTP] Synced: %s\n", buf);
    } else {
        Serial.println("\n[NTP] Sync failed — using RTC drift time");
    }
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  delay(300);

  // WiFi
  wifi_Start();

  // -------------------- Web Route --------------------
  server.on("/relay", HTTP_GET, [](AsyncWebServerRequest* request) {
    // 1. Serve HTML page from PROGMEM
    request->send_P(200, PSTR("text/html"), HTML7, processor7);

    // 2. Trigger LoRa send
    command = 1;
    sendRequested = true;
  });

  server.begin();
 
  // -------------------- TX Task (Core 1) --------------------
  xTaskCreatePinnedToCore(
    txTask,
    "txTask",
    4096,
    NULL,
    1,
    NULL,
    1);

  // -------------------- NTP Task (Core 0, low priority) --------------------
  xTaskCreatePinnedToCore(
    ntpTask,
    "ntpTask",
    4096,
    NULL,
    0,  // lowest priority
    NULL,
    0  // run on Core 0
  );
}

void loop() {}

// -------------------- Template Processor --------------------
String processor7(const String& var) {
  if (var == F("LINK"))
    return linkAddress;
  return String();
}

// -------------------- TX Task --------------------
void txTask(void* parameter) {
    // Wait for WiFi before doing anything
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }


    initRadio();  // init radio on Core 1
    Serial.printf("Radio initialized on Core %d\n", xPortGetCoreID());

    bool ok = initNTP();
    if (!ok) Serial.println("NTP failed, using fallback");

    configTzTime(
      "EST5EDT,M3.2.0/2,M11.1.0/2",
      "pool.ntp.org",
      "time.nist.gov");
      
      waitForNTP();       // ← Block until NTP confirmed

      syncNTP();
    for (;;) {
        if (sendRequested) {
            sendRequested = false;
            sendWORCommand(command);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void waitForBusy() {
  uint32_t start = millis();
  // Wait while BUSY is HIGH (max 1 second safety timeout)
  while (digitalRead(RADIO_BUSY_PIN) == HIGH) {
    if (millis() - start > 1000) break; 
    yield(); 
  }
}

// -------------------- Send WOR + Payload --------------------
bool sendWORCommand(uint8_t command) {
  // 1. WOR wake packet
  LoraPacket wor = {};
  wor.type = 0xAA;
  wor.cmd = command;
  memset(wor.timestr, 0, sizeof(wor.timestr));
  Serial.println("TX: Sending WOR wake packet");

  Serial.printf("TX: wor.type=0x%02X wor.command=%u\n", wor.type, wor.cmd);
  radio.transmit((uint8_t*)&wor, sizeof(LoraPacket));
  
  delay(2000);  // let Rx wake and get ready

  // 2. Real command packet
  LoraPacket msg;
  msg.type = 0xBB;
  msg.cmd = command;
  String ts = getLocalTimestamp();
  ts.toCharArray(msg.timestr, sizeof(msg.timestr));

  Serial.printf("TX: Sending WOR command %u at %s\n", msg.cmd, msg.timestr);

  Serial.printf("TX: msg.type=0x%02X msg.cmd=%u\n", msg.type, msg.cmd);
  radio.transmit((uint8_t*)&msg, sizeof(LoraPacket));  

  // 3. Listen for ACK
  radio.setDio1Action(onDio1);
  ackReceived = false;
  radio.startReceive();
  uint32_t start = millis();

  while (millis() - start < 8000) {
    if (ackReceived) {
      ackReceived = false;
      AckPacket ack;
      int st = radio.readData((uint8_t*)&ack, sizeof(AckPacket));
      if (st == RADIOLIB_ERR_NONE && ack.type == 0xFF) {
        Serial.println("TX: ACK received");
        return true;
      }
      radio.startReceive();
    }
    delay(10);
  }

  Serial.println("TX: No ACK");
  return false;
}
// -------------------- WiFi Start --------------------
void wifi_Start() {
  IPAddress local_IP(192, 168, 12, 27);
  IPAddress gateway(192, 168, 12, 1);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress dns(192, 168, 12, 1);

  WiFi.config(local_IP, gateway, subnet, dns);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi Connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void ntpTask(void* parameter) {
  // Configure NTP once
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  const uint32_t interval = 6UL * 60UL * 60UL * 1000UL;  // 6 hours
  uint32_t lastSync = 0;

  for (;;) {
    uint32_t now = millis();

    if (now - lastSync > interval) {
      if (WiFi.status() == WL_CONNECTED) {

        struct tm timeinfo;
        // VERY small timeout — prevents blocking TX timing
        if (getLocalTime(&timeinfo, 10)) {
          lastSync = now;  // NTP succeeded
        }
        // If NTP fails, skip and try again later
      }
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);  // check once per second
  }
}
