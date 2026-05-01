#include <Arduino.h>
#include <NimBLEDevice.h>

#define DEVICE_NAME  "RoboHand-Controller"
#define SERVICE_UUID "12345678-1234-1234-1234-1234567890ab"
#define CHAR_UUID    "12345678-1234-1234-1234-1234567890ac"

#define NUM_FINGERS 5

const uint8_t flexPins[NUM_FINGERS] = {4, 5, 1, 2, 8};

float smoothVals[NUM_FINGERS];
float alpha = 0.1;

int flexMin[NUM_FINGERS] = {1700, 500, 500, 200, 1700};
int flexMax[NUM_FINGERS] = {3900, 1500, 1500, 1000, 3800};

bool reverseFinger[NUM_FINGERS] = {
  false,  // finger 1
  false,  // finger 2
  true,  // finger 3
  false,  // finger 4
  true   // finger 5
};

int lastAngles[NUM_FINGERS] = {0, 0, 0, 0, 0};

const int DEADBAND = 2;

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 30;

unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 250;

static NimBLEServer* pServer = nullptr;
static NimBLEService* pService = nullptr;
static NimBLECharacteristic* pCharacteristic = nullptr;

bool clientConnected = false;

int flexToAngle(int i, int value) {
  int angle;

  if (reverseFinger[i]) {
    angle = map(value, flexMin[i], flexMax[i], 200, 0);
  } else {
    angle = map(value, flexMin[i], flexMax[i], 0, 200);
  }

  return constrain(angle, 0, 200);
}

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    Serial.printf("Client connected: %s\n", connInfo.getAddress().toString().c_str());
    clientConnected = true;
    pServer->updateConnParams(connInfo.getConnHandle(), 12, 24, 0, 180);
  }

  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    Serial.println("Client disconnected. Restarting advertising...");
    clientConnected = false;
    NimBLEDevice::startAdvertising();
  }
} serverCallbacks;

class CharacteristicCallbacks : public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
    Serial.printf("Characteristic read: %s\n", pCharacteristic->getValue().c_str());
  }

  void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) override {
    Serial.printf("Client subscription value: %u\n", subValue);
  }

  void onStatus(NimBLECharacteristic* pCharacteristic, int code) override {
    Serial.printf("Notify status: %d, %s\n", code, NimBLEUtils::returnCodeToString(code));
  }
} chrCallbacks;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("Starting RoboHand Flex Sender");

  for (int i = 0; i < NUM_FINGERS; i++) {
    smoothVals[i] = analogRead(flexPins[i]);
    lastAngles[i] = flexToAngle(i, (int)smoothVals[i]);
  }

  NimBLEDevice::init(DEVICE_NAME);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(&serverCallbacks);

  pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHAR_UUID,
    NIMBLE_PROPERTY::READ |
    NIMBLE_PROPERTY::NOTIFY
  );

  pCharacteristic->setCallbacks(&chrCallbacks);
  pCharacteristic->setValue("0");

  pService->start();

  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->setName(DEVICE_NAME);
  pAdvertising->addServiceUUID(pService->getUUID());
  pAdvertising->enableScanResponse(true);
  pAdvertising->start();

  Serial.println("Advertising started");
}

void loop() {
  bool changed = false;

  for (int i = 0; i < NUM_FINGERS; i++) {
    int raw = analogRead(flexPins[i]);

    smoothVals[i] = alpha * raw + (1.0 - alpha) * smoothVals[i];

    int angle = flexToAngle(i, (int)smoothVals[i]);

    if (abs(angle - lastAngles[i]) > DEADBAND) {
      lastAngles[i] = angle;
      changed = true;
    }

    if (millis() - lastPrintTime >= PRINT_INTERVAL) {
      Serial.print("Finger ");
      Serial.print(i + 1);
      Serial.print(" GPIO: ");
      Serial.print(flexPins[i]);
      Serial.print("Raw: ");
      Serial.print(raw);
      Serial.print(" Angle:");
      Serial.print(angle);
    }
  }

  if (millis() - lastPrintTime >= PRINT_INTERVAL) {
    Serial.println("----------------------");
    lastPrintTime = millis();
  }

  if (clientConnected && changed && millis() - lastSendTime >= SEND_INTERVAL) {
    char payload[64];

    snprintf(payload, sizeof(payload), "%d,%d,%d,%d,%d", lastAngles[0], lastAngles[1], lastAngles[2], lastAngles[3], lastAngles[4]);

    pCharacteristic->setValue(payload);
    pCharacteristic->notify();

    Serial.print("Sent BLE angle payload: ");
    Serial.println(payload);

    lastSendTime = millis();
  }

  delay(10);
}