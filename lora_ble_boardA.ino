#include <SPI.h>
#include <LoRa.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <GxEPD2_BW.h>
#include <Fonts/FreeMono9pt7b.h>

// ===== E-paper pins (shares SCK/MOSI with LoRa above) =====
#define EPD_CS   0
#define EPD_DC   1
#define EPD_RST  20
#define EPD_BUSY 21

GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> display(GxEPD2_154_D67(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));

// ===== LoRa pins (ESP32-C3) =====
#define LORA_SCK   6
#define LORA_MISO  5
#define LORA_MOSI  4
#define LORA_SS    7
#define LORA_RST   10
#define LORA_DIO0  3
#define LORA_FREQ  433E6

// ===== BLE settings — CHANGE NAME ON THE OTHER BOARD =====
#define DEVICE_NAME "LoRa-BLE-A"

// Standard Nordic UART Service UUIDs — recognized by most generic BLE terminal apps
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// ===== Reliability settings =====
#define ACK_TIMEOUT_MS 2000
#define MAX_RETRIES    3

BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;

// ---------- Outgoing send queue ----------
#define MAX_QUEUE 10
String sendQueue[MAX_QUEUE];
int queueHead = 0, queueTail = 0, queueCount = 0;

void enqueueSend(const String &text) {
  if (queueCount >= MAX_QUEUE) return;
  sendQueue[queueTail] = text;
  queueTail = (queueTail + 1) % MAX_QUEUE;
  queueCount++;
}

bool dequeueSend(String &out) {
  if (queueCount == 0) return false;
  out = sendQueue[queueHead];
  queueHead = (queueHead + 1) % MAX_QUEUE;
  queueCount--;
  return true;
}

// ---------- Pending (in-flight, awaiting ACK) message ----------
uint32_t txCounter = 0;
bool pendingActive = false;
uint32_t pendingId = 0;
String pendingText = "";
unsigned long pendingFirstSent = 0;
unsigned long pendingLastSent = 0;
int pendingRetries = 0;

// ---------- Duplicate detection for received messages ----------
#define DEDUP_HISTORY 10
uint32_t recentReceivedIds[DEDUP_HISTORY];
int dedupHead = 0;
int dedupCount = 0;

bool isDuplicate(uint32_t id) {
  for (int i = 0; i < dedupCount; i++) {
    if (recentReceivedIds[i] == id) return true;
  }
  return false;
}

void rememberReceivedId(uint32_t id) {
  recentReceivedIds[dedupHead] = id;
  dedupHead = (dedupHead + 1) % DEDUP_HISTORY;
  if (dedupCount < DEDUP_HISTORY) dedupCount++;
}

// ---------- E-paper display history ----------
#define DISPLAY_LINES 9
String displayHistory[DISPLAY_LINES];
int displayHistCount = 0;

void redrawDisplay() {
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    display.setFont(&FreeMono9pt7b);
    int y = 14;
    for (int i = 0; i < displayHistCount; i++) {
      display.setCursor(2, y);
      display.print(displayHistory[i]);
      y += 18;
    }
  } while (display.nextPage());
}

void pushDisplayLine(const String &line) {
  if (displayHistCount < DISPLAY_LINES) {
    displayHistory[displayHistCount++] = line;
  } else {
    for (int i = 1; i < DISPLAY_LINES; i++) displayHistory[i - 1] = displayHistory[i];
    displayHistory[DISPLAY_LINES - 1] = line;
  }
  redrawDisplay();
}

// Splits a long line into ~20-char chunks so it fits the 200px-wide panel
void wrapAndPush(const String &line) {
  const int maxChars = 20;
  if (line.length() == 0) {
    pushDisplayLine("");
    return;
  }
  int start = 0;
  while (start < (int)line.length()) {
    int len = min(maxChars, (int)line.length() - start);
    pushDisplayLine(line.substring(start, start + len));
    start += len;
  }
}

// ---------- BLE notify helper ----------
void bleSendLine(const String &text) {
  Serial.println(text); // always mirror to Serial for logging/debugging
  wrapAndPush(text);     // also show on e-paper
  if (deviceConnected) {
    pTxCharacteristic->setValue((uint8_t*)text.c_str(), text.length());
    pTxCharacteristic->notify();
  }
}

// ---------- BLE callbacks ----------
class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("[BLE] Phone connected");
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("[BLE] Phone disconnected, restarting advertising");
    delay(200);
    BLEDevice::startAdvertising();
  }
};

class RxCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = pCharacteristic->getValue().c_str();
    rxValue.trim();
    if (rxValue.length() > 0) {
      enqueueSend(rxValue);
    }
  }
};

// ---------- LoRa TX helpers ----------
void transmitPacket(uint32_t id, const String &text) {
  LoRa.beginPacket();
  LoRa.print("M#");
  LoRa.print(id);
  LoRa.print(":");
  LoRa.print(text);
  LoRa.endPacket();

  Serial.print("[TX] M#");
  Serial.print(id);
  Serial.print(" \"");
  Serial.print(text);
  Serial.println("\"");
}

void sendAck(uint32_t id) {
  LoRa.beginPacket();
  LoRa.print("A#");
  LoRa.print(id);
  LoRa.endPacket();

  Serial.print("[TX] ACK#");
  Serial.println(id);
}

// ---------- Send queue / retry state machine ----------
void processSendQueue() {
  if (!pendingActive) {
    String next;
    if (dequeueSend(next)) {
      txCounter++;
      pendingActive = true;
      pendingId = txCounter;
      pendingText = next;
      pendingFirstSent = millis();
      pendingLastSent = millis();
      pendingRetries = 0;

      bleSendLine("Me: " + pendingText + " [sending...]");
      transmitPacket(pendingId, pendingText);
    }
  } else {
    if (millis() - pendingLastSent > ACK_TIMEOUT_MS) {
      if (pendingRetries < MAX_RETRIES) {
        pendingRetries++;
        pendingLastSent = millis();
        Serial.print("[RETRY] attempt ");
        Serial.println(pendingRetries);
        transmitPacket(pendingId, pendingText);
      } else {
        bleSendLine("Me: " + pendingText + " [FAILED - no response]");
        Serial.println("[FAILED] giving up on message");
        pendingActive = false;
      }
    }
  }
}

// ---------- LoRa RX handling ----------
void checkLoRaReceive() {
  int packetSize = LoRa.parsePacket();
  if (!packetSize) return;

  String data = "";
  while (LoRa.available()) {
    data += (char)LoRa.read();
  }
  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();

  if (data.startsWith("A#")) {
    uint32_t ackId = data.substring(2).toInt();
    Serial.print("[RX] ACK#");
    Serial.println(ackId);

    if (pendingActive && ackId == pendingId) {
      unsigned long rtt = millis() - pendingFirstSent;
      bleSendLine("Me: " + pendingText + " [delivered " + String(rtt) + "ms, " + String(pendingRetries) + " retries]");
      pendingActive = false;
      Serial.print("[DELIVERED] RTT=");
      Serial.print(rtt);
      Serial.println("ms");
    }
  } else if (data.startsWith("M#")) {
    int sep = data.indexOf(':');
    if (sep > 2) {
      uint32_t msgId = data.substring(2, sep).toInt();
      String text = data.substring(sep + 1);

      sendAck(msgId); // always ACK, even duplicates, in case our earlier ACK was lost

      if (!isDuplicate(msgId)) {
        rememberReceivedId(msgId);

        Serial.print("[RX] M#");
        Serial.print(msgId);
        Serial.print(" \"");
        Serial.print(text);
        Serial.print("\" RSSI=");
        Serial.print(rssi);
        Serial.print(" SNR=");
        Serial.println(snr);

        bleSendLine("Them: " + text + " [RSSI " + String(rssi) + "dBm, SNR " + String(snr, 1) + "dB]");
      } else {
        Serial.print("[RX] duplicate M#");
        Serial.println(msgId);
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("===== LoRa Reliable Chat over BLE =====");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  Serial.println("Starting LoRa...");
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa initialization FAILED!");
    while (1);
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0xF3);
  LoRa.enableCrc();
  Serial.println("LoRa initialization SUCCESS!");

  // ---- E-paper setup (shares the SPI bus already started above) ----
  display.init(115200, true, 2, false);
  display.setRotation(1);
  redrawDisplay(); // clears to blank white screen
  pushDisplayLine("LoRa Chat Ready");

  // ---- BLE setup ----
  BLEDevice::init(DEVICE_NAME);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_RX,
                        BLECharacteristic::PROPERTY_WRITE
                      );
  pRxCharacteristic->setCallbacks(new RxCallbacks());

  pService->start();
  pServer->getAdvertising()->start();

  Serial.print("BLE advertising as: ");
  Serial.println(DEVICE_NAME);
  Serial.println("Connect using a BLE terminal app (e.g. Serial Bluetooth Terminal / nRF Connect)");
}

void loop() {
  checkLoRaReceive();
  processSendQueue();
}
