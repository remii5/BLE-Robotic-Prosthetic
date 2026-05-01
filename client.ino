#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define DEVICE_NAME  "RoboHand-ServoClient"
#define SERVICE_UUID "12345678-1234-1234-1234-1234567890ab"
#define CHAR_UUID    "12345678-1234-1234-1234-1234567890ac"

#define NUM_FINGERS 5

#define SDA_PIN 6
#define SCL_PIN 7

#define SERVO_FREQ 50
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2400

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

const uint8_t servoChannels[NUM_FINGERS] = {0, 1, 2, 3, 4};

int currentAngles[NUM_FINGERS] = {90, 90, 90, 90, 90};
int targetAngles[NUM_FINGERS]  = {90, 90, 90, 90, 90};

const int DEADBAND = 2;
const int MAX_STEP = 2;

bool isConnected = false;

static const NimBLEAdvertisedDevice* advDevice = nullptr;
static bool doConnect = false;
static uint32_t scanTimeMs = 5000;

NimBLERemoteService* pSvc = nullptr;
NimBLERemoteCharacteristic* pChr = nullptr;

int angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);

  int pulseUS = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  int periodUS = 1000000 / SERVO_FREQ;
  int pulse = (pulseUS * 4096) / periodUS;

  return pulse;
}

void writeServoAngle(int index, int angle) {
  if (index < 0 || index >= NUM_FINGERS) return;
  pwm.setPWM(servoChannels[index], 0, angleToPulse(angle));
}

void testServosAtStartup() {
  Serial.println("Testing PCA9685 servos...");

  for (int i = 0; i < NUM_FINGERS; i++) {
    writeServoAngle(i, 0);
  }
  delay(700);

  for (int i = 0; i < NUM_FINGERS; i++) {
    writeServoAngle(i, 90);
  }
  delay(700);

  for (int i = 0; i < NUM_FINGERS; i++) {
    writeServoAngle(i, 180);
  }
  delay(700);

  for (int i = 0; i < NUM_FINGERS; i++) {
    writeServoAngle(i, 90);
  }
  delay(700);

  Serial.println("Servo startup test complete.");
}

class ClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pClient) override {
    Serial.println("Connected to flex controller");
    isConnected = true;
    NimBLEDevice::getScan()->stop();
  }

  void onDisconnect(NimBLEClient* pClient, int reason) override {
    Serial.printf("Disconnected, reason = %d. Restarting scan...\n", reason);
    isConnected = false;
    pSvc = nullptr;
    pChr = nullptr;
    NimBLEDevice::getScan()->start(scanTimeMs, false, true);
  }
} clientCallbacks;

class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
    if (advertisedDevice->isAdvertisingService(NimBLEUUID(SERVICE_UUID))) {
      Serial.println("Found RoboHand controller service");
      Serial.printf("Controller: %s\n", advertisedDevice->toString().c_str());

      NimBLEDevice::getScan()->stop();
      advDevice = advertisedDevice;
      doConnect = true;
    }
  }

  void onScanEnd(const NimBLEScanResults& results, int reason) override {
    if (!isConnected && !doConnect) {
      Serial.println("Scan ended. Restarting scan...");
      NimBLEDevice::getScan()->start(scanTimeMs, false, true);
    }
  }
} scanCallbacks;

void parseAnglePayload(const char* payload) {
  int values[NUM_FINGERS];

  int parsed = sscanf(payload, "%d,%d,%d,%d,%d",
                      &values[0], &values[1], &values[2],
                      &values[3], &values[4]);

  if (parsed != NUM_FINGERS) {
    Serial.print("Bad payload: ");
    Serial.println(payload);
    return;
  }

  for (int i = 0; i < NUM_FINGERS; i++) {
    values[i] = constrain(values[i], 0, 180);

    if (abs(values[i] - targetAngles[i]) > DEADBAND) {
      targetAngles[i] = values[i];
    }
  }

  Serial.print("Received target angles: ");
  for (int i = 0; i < NUM_FINGERS; i++) {
    Serial.print(targetAngles[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void handleServoAngles(NimBLERemoteCharacteristic* pRemoteCharacteristic,
                       uint8_t* pData, size_t length, bool isNotify) {
  char payload[64];

  if (length >= sizeof(payload)) {
    length = sizeof(payload) - 1;
  }

  memcpy(payload, pData, length);
  payload[length] = '\0';

  Serial.print("Notification payload: ");
  Serial.println(payload);

  parseAnglePayload(payload);
}

bool connectToServer() {
  NimBLEClient* pClient = nullptr;

  if (NimBLEDevice::getCreatedClientCount()) {
    pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());

    if (pClient) {
      if (!pClient->connect(advDevice, false)) {
        Serial.println("Reconnect failed");
        return false;
      }
    } else {
      pClient = NimBLEDevice::getDisconnectedClient();
    }
  }

  if (!pClient) {
    if (NimBLEDevice::getCreatedClientCount() >= NIMBLE_MAX_CONNECTIONS) {
      Serial.println("Max BLE clients reached");
      return false;
    }

    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(&clientCallbacks, false);
    pClient->setConnectionParams(24, 48, 0, 180);
    pClient->setConnectTimeout(5000);

    if (!pClient->connect(advDevice)) {
      NimBLEDevice::deleteClient(pClient);
      Serial.println("Failed to connect, deleted client");
      return false;
    }
  }

  if (!pClient->isConnected()) {
    if (!pClient->connect(advDevice)) {
      Serial.println("Failed to connect");
      return false;
    }
  }

  Serial.printf("Connected to: %s RSSI: %d\n",
                pClient->getPeerAddress().toString().c_str(),
                pClient->getRssi());

  pSvc = pClient->getService(SERVICE_UUID);

  if (!pSvc) {
    Serial.println("Service not found");
    return false;
  }

  pChr = pSvc->getCharacteristic(CHAR_UUID);

  if (!pChr) {
    Serial.println("Characteristic not found");
    return false;
  }

  if (!pChr->subscribe(true, handleServoAngles)) {
    Serial.println("Failed to subscribe");
    return false;
  }

  Serial.println("Subscribed to servo angle notifications");
  return true;
}

void updateServos() {
  static unsigned long lastDebug = 0;

  for (int i = 0; i < NUM_FINGERS; i++) {
    if (currentAngles[i] < targetAngles[i]) {
      currentAngles[i] += MAX_STEP;
      if (currentAngles[i] > targetAngles[i]) {
        currentAngles[i] = targetAngles[i];
      }
    } else if (currentAngles[i] > targetAngles[i]) {
      currentAngles[i] -= MAX_STEP;
      if (currentAngles[i] < targetAngles[i]) {
        currentAngles[i] = targetAngles[i];
      }
    }

    writeServoAngle(i, currentAngles[i]);
  }

  if (millis() - lastDebug >= 500) {
    Serial.print("Current: ");
    for (int i = 0; i < NUM_FINGERS; i++) {
      Serial.print(currentAngles[i]);
      Serial.print(" ");
    }

    Serial.print("| Target: ");
    for (int i = 0; i < NUM_FINGERS; i++) {
      Serial.print(targetAngles[i]);
      Serial.print(" ");
    }

    Serial.println();
    lastDebug = millis();
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("Starting RoboHand BLE Servo Client with PCA9685");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Wire.beginTransmission(0x40);
  byte error = Wire.endTransmission();

  /* testing connection to servo driver
  if (error == 0) {
    Serial.println("PCA9685 found at 0x40");
  } else {
    Serial.print("PCA9685 not found. I2C error: ");
    Serial.println(error);
  }
  */

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  for (int i = 0; i < NUM_FINGERS; i++) {
    writeServoAngle(i, currentAngles[i]);
  }

  //testServosAtStartup();

  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(0);

  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(&scanCallbacks, false);
  pScan->setInterval(100);
  pScan->setWindow(100);
  pScan->setActiveScan(true);
  pScan->start(scanTimeMs);

  Serial.println("Scanning for RoboHand controller...");
}

void loop() {
  if (doConnect) {
    doConnect = false;

    if (connectToServer()) {
      NimBLEDevice::getScan()->stop();
      Serial.println("Connected. Waiting for angle notifications.");
    } else {
      Serial.println("Connection failed. Restarting scan.");
      NimBLEDevice::getScan()->start(scanTimeMs, false, true);
    }
  }

  updateServos();

  delay(15);
}