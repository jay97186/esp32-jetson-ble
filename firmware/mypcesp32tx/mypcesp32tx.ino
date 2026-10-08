#include <Adafruit_NeoPixel.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


// ============================================================
// LED setting
// ============================================================
#define PIN         48
#define NUMPIXELS   1
#define BRIGHTNESS 30

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


// ============================================================
// BLE setting
// ============================================================
#define DEVICE_NAME "ESP32_S3_TX"

#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define TX_CHAR_UUID        "12345678-1234-1234-1234-1234567890ac"  // ESP32 -> Jetson
#define RX_CHAR_UUID        "12345678-1234-1234-1234-1234567890ad"  // Jetson -> ESP32

BLECharacteristic *txCharacteristic;
BLECharacteristic *rxCharacteristic;

bool deviceConnected = false;


// ============================================================
// Data setting
// ============================================================
// ?ㄐ?臭誑?曇???8 bytes ??銝?
// ESP32 ???????瘥? 8 bytes
const char dataText[] = "I don't have an age; I'm Codex, an AI.";


// ============================================================
// Protocol setting
// ============================================================
const char preambleBits[] = "1111100110101";
#define PREAMBLE_BITS 13

#define HEADER_BYTES   1
#define PAYLOAD_BYTES  8
#define CRC_BYTES      1

#define RAW_BYTES      (HEADER_BYTES + PAYLOAD_BYTES + CRC_BYTES)  // 10 bytes
#define RAW_BITS       (RAW_BYTES * 8)                              // 80 bits

#define NIBBLE_COUNT   (RAW_BITS / 4)                               // 20 nibbles
#define HAMMING_BITS_PER_CODEWORD 7
#define ENCODED_BITS   (NIBBLE_COUNT * HAMMING_BITS_PER_CODEWORD)   // 140 bits

#define TX_BITS_TOTAL  (PREAMBLE_BITS + ENCODED_BITS)               // 153 bits

char txBits[TX_BITS_TOTAL + 1];


// ============================================================
// Timing setting
// ============================================================
// Camera = 60 fps
// 1 frame = 1 bit
const unsigned long BIT_INTERVAL_US = 16667;

// ?喳?銝???憭? ACK 撟暹神蝘?
const unsigned long ACK_TIMEOUT_MS = 5000;


// ============================================================
// ACK state
// ============================================================
volatile bool ackReceived = false;
volatile int ackPacketId = -1;

volatile bool promptReceived = false;
String latestPrompt = "";

void notifyStatus(String msg);


// ============================================================
// Packet state
// ============================================================
int totalPackets = 0;
int currentPacketId = 0;
bool allDone = false;


// ============================================================
// BLE callbacks
// ============================================================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("[BLE] Jetson connected.");
  }

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("[BLE] Jetson disconnected.");
    pServer->startAdvertising();
  }
};


class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();

    value.trim();

    if (value.length() > 0) {
      Serial.print("[BLE] Received from Jetson: ");
      Serial.println(value);

      if (value.startsWith("PROMPT:")) {
        latestPrompt = value.substring(7);
        promptReceived = true;
        allDone = false;
        currentPacketId = 0;
        ackReceived = false;
        ackPacketId = -1;

        Serial.print("[PROMPT] Received prompt = ");
        Serial.println(latestPrompt);
        notifyStatus("PROMPT_ACK:RECEIVED\n");
        return;
      }

      // ACK format: ACK:0, ACK:1, ACK:2...
      if (value.startsWith("ACK:")) {
        int id = value.substring(4).toInt();

        ackPacketId = id;
        ackReceived = true;

        Serial.print("[BLE] Parsed ACK packet_id = ");
        Serial.println(id);
      }
    }
  }
};


// ============================================================
// LED output
// ============================================================
void setBit(char b) {
  if (b == '1') {
    pixels.setPixelColor(0, pixels.Color(255, 255, 255));
  } else {
    pixels.clear();
  }

  pixels.show();
}


void ledOff() {
  pixels.clear();
  pixels.show();
}


// ============================================================
// CRC-8
// ============================================================
// poly = 0x07
// init = 0x00
uint8_t crc8(const uint8_t *data, int len) {
  uint8_t crc = 0x00;
  const uint8_t poly = 0x07;

  for (int i = 0; i < len; i++) {
    crc ^= data[i];

    for (int b = 0; b < 8; b++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ poly;
      } else {
        crc = (crc << 1);
      }
    }
  }

  return crc;
}


// ============================================================
// Header
// ============================================================
// header 1 byte:
// bit 7~4 : packet_id
// bit 3   : last_packet
// bit 2~0 : payload_len
//
// 雿?刻身閮?payload ?箏? 8 bytes嚗?隞?len_field ?箏???0
// 0 means 8 bytes
uint8_t buildHeader(uint8_t packet_id, bool last_packet) {
  packet_id &= 0x0F;

  uint8_t header = 0;
  header |= (packet_id << 4);
  header |= (last_packet ? 1 : 0) << 3;
  header |= 0;   // len_field = 0 means payload = 8 bytes

  return header;
}


// ============================================================
// Hamming(7,4) encoder
// ============================================================
// codeword = d1 d2 d3 d4 p1 p2 p3
void hamming74Encode(uint8_t nibble, char out[7]) {
  uint8_t d1 = (nibble >> 3) & 1;
  uint8_t d2 = (nibble >> 2) & 1;
  uint8_t d3 = (nibble >> 1) & 1;
  uint8_t d4 = (nibble >> 0) & 1;

  uint8_t p1 = d1 ^ d2 ^ d4;
  uint8_t p2 = d1 ^ d3 ^ d4;
  uint8_t p3 = d2 ^ d3 ^ d4;

  out[0] = d1 ? '1' : '0';
  out[1] = d2 ? '1' : '0';
  out[2] = d3 ? '1' : '0';
  out[3] = d4 ? '1' : '0';
  out[4] = p1 ? '1' : '0';
  out[5] = p2 ? '1' : '0';
  out[6] = p3 ? '1' : '0';
}


// ============================================================
// Debug helper
// ============================================================
void printByteBits(uint8_t value) {
  for (int i = 7; i >= 0; i--) {
    Serial.print((value >> i) & 1);
  }
}


// ============================================================
// Build one packet
// ============================================================
void buildTxBitsForPacket(int packet_id) {
  uint8_t raw[RAW_BYTES];
  uint8_t payload[PAYLOAD_BYTES];

  int textLen = strlen(dataText);

  int startIndex = packet_id * PAYLOAD_BYTES;

  bool last_packet = false;

  if (startIndex + PAYLOAD_BYTES >= textLen) {
    last_packet = true;
  }

  // payload ?箏? 8 bytes嚗?頞唾? 0x00
  for (int i = 0; i < PAYLOAD_BYTES; i++) {
    int textIndex = startIndex + i;

    if (textIndex < textLen) {
      payload[i] = (uint8_t)dataText[textIndex];
    } else {
      payload[i] = 0x00;
    }
  }

  uint8_t header = buildHeader(packet_id, last_packet);

  raw[0] = header;

  for (int i = 0; i < PAYLOAD_BYTES; i++) {
    raw[1 + i] = payload[i];
  }

  uint8_t crc = crc8(raw, HEADER_BYTES + PAYLOAD_BYTES);
  raw[HEADER_BYTES + PAYLOAD_BYTES] = crc;

  // Hamming encode
  char encoded[NIBBLE_COUNT][HAMMING_BITS_PER_CODEWORD];

  for (int i = 0; i < RAW_BYTES; i++) {
    uint8_t value = raw[i];

    uint8_t highNibble = (value >> 4) & 0x0F;
    uint8_t lowNibble  = value & 0x0F;

    hamming74Encode(highNibble, encoded[i * 2]);
    hamming74Encode(lowNibble,  encoded[i * 2 + 1]);
  }

  int idx = 0;

  // Add preamble
  for (int i = 0; i < PREAMBLE_BITS; i++) {
    txBits[idx++] = preambleBits[i];
  }

  // Interleave
  for (int col = 0; col < HAMMING_BITS_PER_CODEWORD; col++) {
    for (int row = 0; row < NIBBLE_COUNT; row++) {
      txBits[idx++] = encoded[row][col];
    }
  }

  txBits[idx] = '\0';

  // Debug
  Serial.println();
  Serial.println("========== Build Packet ==========");
  Serial.print("Packet ID = ");
  Serial.println(packet_id);

  Serial.print("Last packet = ");
  Serial.println(last_packet ? "true" : "false");

  Serial.print("Header = 0b");
  printByteBits(header);
  Serial.print(" = 0x");
  if (header < 0x10) Serial.print("0");
  Serial.println(header, HEX);

  Serial.print("Payload ASCII = ");
  for (int i = 0; i < PAYLOAD_BYTES; i++) {
    if (payload[i] >= 32 && payload[i] <= 126) {
      Serial.print((char)payload[i]);
    } else {
      Serial.print(".");
    }
  }
  Serial.println();

  Serial.print("Payload hex = ");
  for (int i = 0; i < PAYLOAD_BYTES; i++) {
    if (payload[i] < 0x10) Serial.print("0");
    Serial.print(payload[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  Serial.print("CRC-8 = 0x");
  if (crc < 0x10) Serial.print("0");
  Serial.println(crc, HEX);

  Serial.print("TX bits total = ");
  Serial.println(strlen(txBits));
  Serial.println("==================================");
}


// ============================================================
// Send current packet by LED
// ============================================================
void sendCurrentPacketBits() {
  Serial.println();
  Serial.print("[TX] Sending packet ");
  Serial.println(currentPacketId);

  for (int i = 0; txBits[i] != '\0'; i++) {
    unsigned long startTime = micros();

    setBit(txBits[i]);

    while (micros() - startTime < BIT_INTERVAL_US) {
      // wait
    }
  }

  ledOff();

  Serial.print("[TX] Packet ");
  Serial.print(currentPacketId);
  Serial.println(" sent.");
}


// ============================================================
// Wait ACK
// ============================================================
bool waitAckForPacket(int expectedPacketId, unsigned long timeoutMs) {
  ackReceived = false;
  ackPacketId = -1;

  unsigned long startTime = millis();

  Serial.print("[ACK] Waiting for ACK:");
  Serial.println(expectedPacketId);

  while (millis() - startTime < timeoutMs) {
    if (ackReceived) {
      noInterrupts();
      int receivedId = ackPacketId;
      ackReceived = false;
      interrupts();

      Serial.print("[ACK] Got ACK:");
      Serial.println(receivedId);

      if (receivedId == expectedPacketId) {
        Serial.println("[ACK] ACK matched.");
        return true;
      } else {
        Serial.println("[ACK] ACK id mismatch. Continue waiting.");
      }
    }

    delay(10);
  }

  Serial.println("[ACK] Timeout. Need resend.");
  return false;
}


// ============================================================
// BLE notify helper
// ============================================================
void notifyStatus(String msg) {
  if (deviceConnected && txCharacteristic != nullptr) {
    txCharacteristic->setValue(msg.c_str());
    txCharacteristic->notify();
  }
}


// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting ESP32-S3 BLE + LED TX...");

  pixels.begin();
  pixels.setBrightness(BRIGHTNESS);
  ledOff();

  int textLen = strlen(dataText);
  totalPackets = (textLen + PAYLOAD_BYTES - 1) / PAYLOAD_BYTES;

  if (totalPackets == 0) {
    totalPackets = 1;
  }

  if (totalPackets > 16) {
    Serial.println("[WARNING] packet_id only supports 0~15.");
    Serial.println("[WARNING] Data longer than 16 packets will overflow packet_id.");
  }

  Serial.print("Data text = ");
  Serial.println(dataText);

  Serial.print("Text length = ");
  Serial.println(textLen);

  Serial.print("Total packets = ");
  Serial.println(totalPackets);

  BLEDevice::init(DEVICE_NAME);

  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new MyServerCallbacks());

  BLEService *service = server->createService(SERVICE_UUID);

  // ESP32 -> Jetson notify
  txCharacteristic = service->createCharacteristic(
    TX_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  txCharacteristic->addDescriptor(new BLE2902());

  // Jetson -> ESP32 write
  rxCharacteristic = service->createCharacteristic(
    RX_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  rxCharacteristic->setCallbacks(new RxCallbacks());

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();

  Serial.println("BLE started.");
  Serial.println("Device name: ESP32_S3_TX");
  Serial.println("Waiting for Jetson connection...");
}


// ============================================================
// Main loop
// ============================================================
void loop() {
  if (allDone) {
    ledOff();
    delay(1000);
    return;
  }

  if (!deviceConnected) {
    delay(100);
    return;
  }

  if (!promptReceived) {
    ledOff();
    delay(100);
    return;
  }

  if (currentPacketId >= totalPackets) {
    Serial.println("[DONE] All packets sent and ACKed.");
    notifyStatus("TX_DONE\n");
    allDone = true;
    return;
  }

  buildTxBitsForPacket(currentPacketId);

  sendCurrentPacketBits();

  bool ackOk = waitAckForPacket(currentPacketId, ACK_TIMEOUT_MS);

  if (ackOk) {
    currentPacketId++;

    if (currentPacketId >= totalPackets) {
      Serial.println("[DONE] Last packet ACKed.");
      notifyStatus("TX_DONE\n");
      allDone = true;
    } else {
      Serial.println("[TX] Go to next packet.");
      delay(300);
    }
  } else {
    Serial.println("[TX] Resend same packet.");
    delay(300);
  }
}
