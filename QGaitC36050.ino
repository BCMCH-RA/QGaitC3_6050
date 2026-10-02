/* ============================================================================
GaitIMU v12 — ESP32-C3 + MPU6050
200 Hz BLE IMU streamer, battery stub, protocol v2

Target:
  ESP32-C3 + MPU6050 over I2C

Implements:
  Option A: up to 8 samples per BLE packet
  Option B: binary int16 IMU samples
  Option C: one timestamp per packet
  Option D: packet sequence + global sample sequence
  Periodic time-resync support from browser

BLE protocol v2:
  Service:
    9f000001-8c5a-4e1f-9a7b-3d6e5f0a1b2c

  Characteristics:
    CMD  : 9f000002-...  write       0x01 start, 0x02 stop, 0x03 [N] spp
    TIME : 9f000003-...  write+notify
    DATA : 9f000004-...  notify
    ID   : 9f000005-...  read

  TIME write format from browser:
    uint32 unix_seconds
    uint16 unix_milliseconds 0..999

  TIME notify ACK format from module:
    uint32 device_time_ms
    uint32 unix_ms_low32

  DATA packet format v2:
    uint8  protocol_version = 2
    uint16 packet_seq
    uint32 first_sample_seq
    uint32 first_device_time_ms
    uint8  sample_count
    uint8  flags
    int16  sample0_ax
    int16  sample0_ay
    int16  sample0_az
    int16  sample0_gx
    int16  sample0_gy
    int16  sample0_gz
    ...

Scaling:
  ±8 g      -> g   = raw * 0.000244140625
  ±1000 dps -> dps = raw * 0.030517578125

MPU6050 settings used:
  Accel ±8 g
  Gyro  ±1000 dps
  1 kHz internal sample rate, DLPF ~42 Hz
  Browser/firmware still extracts 200 Hz by polling

Notes:
  Battery service is included but reports fixed 100% unless you add ADC code.
============================================================================ */

#include <Arduino.h>
#include <Wire.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_gatt_defs.h>

#include <string.h>

/* ───────── USER CONFIG — set per module before flashing ───────── */
#define MODULE_ID "RightLeg"   /* "RightLeg" | "LeftLeg" | "Torso" */
/* ──────────────────────────────────────────────────────────────── */

#define DEVICE_NAME "GaitIMU-" MODULE_ID

/* ───────── I2C pins ─────────
   Adjust for your ESP32-C3 board.

   Common generic ESP32-C3 dev kit:
     SDA = GPIO8
     SCL = GPIO9

   Seeed XIAO ESP32-C3 is often:
     SDA = GPIO6
     SCL = GPIO7
*/
#define I2C_SDA 8
#define I2C_SCL 9

/* Sampling / packetisation */
#define SAMPLE_RATE_HZ           200
#define SAMPLE_PERIOD_MS         (1000 / SAMPLE_RATE_HZ)
#define MAX_SAMPLES_PER_PACKET   8
#define DEFAULT_SAMPLES_PER_PACKET 8

#define HEADER_LEN               13
#define MAX_PACKET_LEN           (HEADER_LEN + 12 * MAX_SAMPLES_PER_PACKET)

/* BLE UUIDs — must match index.html */
#define GAIT_SERVICE_UUID "9f000001-8c5a-4e1f-9a7b-3d6e5f0a1b2c"
#define CMD_CHAR_UUID     "9f000002-8c5a-4e1f-9a7b-3d6e5f0a1b2c"
#define TIME_CHAR_UUID    "9f000003-8c5a-4e1f-9a7b-3d6e5f0a1b2c"
#define DATA_CHAR_UUID    "9f000004-8c5a-4e1f-9a7b-3d6e5f0a1b2c"
#define ID_CHAR_UUID      "9f000005-8c5a-4e1f-9a7b-3d6e5f0a1b2c"

#define CMD_START     0x01
#define CMD_STOP      0x02
#define CMD_SET_SPP   0x03

/* MPU6050 registers */
#define MPU_ADDR_DEFAULT     0x68
#define MPU_REG_WHO_AM_I     0x75
#define MPU_REG_SMPLRT_DIV   0x19
#define MPU_REG_CONFIG       0x1A
#define MPU_REG_GYRO_CONFIG  0x18
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_ACCEL_XOUT_H 0x3B

/* BLE objects */
BLEServer*         pServer = nullptr;
BLEService*        pService = nullptr;
BLEService*        pBatteryService = nullptr;

BLECharacteristic* cmdChar = nullptr;
BLECharacteristic* timeChar = nullptr;
BLECharacteristic* dataChar = nullptr;
BLECharacteristic* idChar = nullptr;
BLECharacteristic* battChar = nullptr;

/* MPU6050 detected address */
static uint8_t mpuAddr = 0;

/* Connection state */
volatile bool bleConnected = false;

/* Command flags from BLE callbacks */
volatile bool pendingStart = false;
volatile bool pendingStop = false;
volatile bool pendingSetSpp = false;
volatile uint8_t pendingSpp = DEFAULT_SAMPLES_PER_PACKET;

/* Streaming state */
volatile bool streaming = false;

uint64_t syncUnixMs = 0;
uint32_t syncDeviceMs = 0;
bool timeSynced = false;

uint32_t nextSampleMs = 0;
uint8_t samplesPerPacket = DEFAULT_SAMPLES_PER_PACKET;

uint16_t packetSeq = 0;
volatile uint8_t packetFill = 0;

uint32_t packetFirstSampleSeq = 0;
uint32_t packetT0 = 0;
uint8_t packetFlags = 0;

uint32_t sampleSeq = 0;

uint8_t packetBuf[MAX_PACKET_LEN];

uint32_t statLast = 0;
uint32_t pktCount = 0;
uint32_t smpCount = 0;
uint32_t i2cErrors = 0;
bool lastReadError = false;

uint32_t batLast = 0;
uint8_t batteryPct = 100;
bool batteryLow = false;

bool imuOk = false;

/* ═══════════════════════════════════════════════════════════════
   Low-level MPU6050 I2C helpers
   ═══════════════════════════════════════════════════════════════ */

static void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.endTransmission(false);

  if (Wire.requestFrom(mpuAddr, (uint8_t)1) != 1) {
    return 0xFF;
  }

  return (uint8_t)Wire.read();
}

static bool detectMPU(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(MPU_REG_WHO_AM_I);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(addr, (uint8_t)1) != 1) {
    return false;
  }

  uint8_t who = (uint8_t)Wire.read();

  /*
     MPU6050 WHO_AM_I is normally 0x68 when AD0 low.
     Some modules/clones may report the address or nearby values.
     Accept 0x68 or the addressed value.
  */
  return (who == 0x68 || who == addr);
}

static bool readBurst14(uint8_t* dst) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(MPU_REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);

  if (Wire.requestFrom(mpuAddr, (uint8_t)14) != 14) {
    return false;
  }

  for (int i = 0; i < 14; i++) {
    if (Wire.available() < 1) {
      return false;
    }
    dst[i] = (uint8_t)Wire.read();
  }

  return true;
}

/*
   MPU6050 14-byte burst starting at 0x3B:

     ACCEL_XOUT_H/L
     ACCEL_YOUT_H/L
     ACCEL_ZOUT_H/L
     TEMP_OUT_H/L
     GYRO_XOUT_H/L
     GYRO_YOUT_H/L
     GYRO_ZOUT_H/L

   Returned sample layout matches firmware packet order:
     acc[3]
     gyr[3]
*/
static bool hwReadSample(int16_t acc[3], int16_t gyr[3]) {
  uint8_t b[14];

  if (!readBurst14(b)) {
    return false;
  }

  for (int i = 0; i < 3; i++) {
    acc[i] = (int16_t)(((uint16_t)b[2 * i] << 8) | (uint16_t)b[2 * i + 1]);
  }

  /* Gyro starts after accel 6 bytes + temp 2 bytes = byte 8 */
  for (int i = 0; i < 3; i++) {
    gyr[i] = (int16_t)(((uint16_t)b[8 + 2 * i] << 8) | (uint16_t)b[9 + 2 * i]);
  }

  return true;
}

static bool imuInit() {
  Wire.begin(I2C_SDA, I2C_SCL, 400000);

  uint8_t addrs[2] = {0x68, 0x69};
  mpuAddr = 0;

  for (uint8_t i = 0; i < 2; i++) {
    if (detectMPU(addrs[i])) {
      mpuAddr = addrs[i];
      break;
    }
  }

  if (mpuAddr == 0) {
    Serial.println("[IMU] MPU6050 not found at 0x68 or 0x69");
    return false;
  }

  /*
     Wake MPU6050.
     First clear sleep, then select PLL with X gyro reference.
  */
  writeReg(MPU_REG_PWR_MGMT_1, 0x00);
  delay(10);

  writeReg(MPU_REG_PWR_MGMT_1, 0x01);
  delay(10);

  /*
     Sample rate divider:
       1 kHz / (1 + 0) = 1 kHz internal output

     CONFIG = 0x03:
       DLPF_CFG = 3, roughly 42 Hz bandwidth
       Good compromise for 200 Hz gait sampling.
  */
  writeReg(MPU_REG_SMPLRT_DIV, 0x00);
  writeReg(MPU_REG_CONFIG, 0x03);

  /*
     GYRO_CONFIG = 0x10:
       FS_SEL = 2 -> ±1000 dps

     ACCEL_CONFIG = 0x10:
       AFS_SEL = 2 -> ±8 g
  */
  writeReg(MPU_REG_GYRO_CONFIG, 0x10);
  writeReg(MPU_REG_ACCEL_CONFIG, 0x10);

  uint8_t who = readReg(MPU_REG_WHO_AM_I);
  uint8_t gc = readReg(MPU_REG_GYRO_CONFIG);
  uint8_t ac = readReg(MPU_REG_ACCEL_CONFIG);
  uint8_t cfg = readReg(MPU_REG_CONFIG);

  Serial.printf("[IMU] MPU6050 found at 0x%02X, WHO_AM_I=0x%02X\n", mpuAddr, who);
  Serial.printf("[IMU] CONFIG=0x%02X GYRO_CONFIG=0x%02X ACCEL_CONFIG=0x%02X\n", cfg, gc, ac);

  return true;
}

/* ═══════════════════════════════════════════════════════════════
   Packet build / send
   ═══════════════════════════════════════════════════════════════ */

static void flushPacket() {
  if (packetFill == 0) {
    return;
  }

  packetBuf[0] = 2; /* protocol version */
  packetBuf[1] = (uint8_t)(packetSeq & 0xFF);
  packetBuf[2] = (uint8_t)(packetSeq >> 8);

  memcpy(&packetBuf[3], &packetFirstSampleSeq, 4);
  memcpy(&packetBuf[7], &packetT0, 4);

  packetBuf[11] = packetFill;
  packetBuf[12] = packetFlags;

  size_t len = HEADER_LEN + 12 * packetFill;

  if (bleConnected && dataChar != nullptr) {
    dataChar->setValue(packetBuf, len);
    dataChar->notify();
  }

  packetSeq++;
  packetFill = 0;
  pktCount++;
}

static void addSample(uint32_t sampleMs) {
  int16_t acc[3];
  int16_t gyr[3];

  if (!hwReadSample(acc, gyr)) {
    i2cErrors++;
    lastReadError = true;
    return;
  }

  lastReadError = false;

  if (packetFill == 0) {
    packetT0 = sampleMs;
    packetFirstSampleSeq = sampleSeq;
    packetFlags = 0;

    if (timeSynced) {
      packetFlags |= 0x01;
    }

    if (lastReadError) {
      packetFlags |= 0x02;
    }

    if (batteryLow) {
      packetFlags |= 0x04;
    }
  }

  uint8_t* p = &packetBuf[HEADER_LEN + 12 * packetFill];

  memcpy(p, acc, 6);
  memcpy(p + 6, gyr, 6);

  packetFill++;
  sampleSeq++;
  smpCount++;

  if (packetFill >= samplesPerPacket) {
    flushPacket();
  }
}

/* ═══════════════════════════════════════════════════════════════
   BLE callbacks
   ═══════════════════════════════════════════════════════════════ */

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) {
    bleConnected = true;
    Serial.println("[BLE] central connected");
  }

  void onDisconnect(BLEServer* server) {
    bleConnected = false;
    streaming = false;
    packetFill = 0;

    Serial.println("[BLE] central disconnected — advertising again");
    BLEDevice::startAdvertising();
  }
};

class CmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    std::string v = pCharacteristic->getValue();

    if (v.length() < 1) {
      return;
    }

    uint8_t b0 = (uint8_t)v[0];
    uint8_t b1 = (v.length() > 1) ? (uint8_t)v[1] : 0;

    if (b0 == CMD_START) {
      pendingStart = true;
    } else if (b0 == CMD_STOP) {
      pendingStop = true;
    } else if (b0 == CMD_SET_SPP) {
      pendingSpp = b1;
      pendingSetSpp = true;
    }
  }
};

class TimeCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    std::string v = pCharacteristic->getValue();

    if (v.length() < 6) {
      return;
    }

    const uint8_t* p = reinterpret_cast<const uint8_t*>(v.data());

    uint32_t sec = 0;
    uint16_t ms = 0;

    memcpy(&sec, &p[0], 4);
    memcpy(&ms, &p[4], 2);

    syncUnixMs = ((uint64_t)sec * 1000ULL) + (uint64_t)ms;
    syncDeviceMs = millis();
    timeSynced = true;

    uint8_t ack[8];
    uint32_t unixLow = (uint32_t)(syncUnixMs & 0xFFFFFFFF);

    memcpy(&ack[0], &syncDeviceMs, 4);
    memcpy(&ack[4], &unixLow, 4);

    if (timeChar != nullptr) {
      timeChar->setValue(ack, 8);
      timeChar->notify();
    }
  }
};

/* ═══════════════════════════════════════════════════════════════
   setup / loop
   ═══════════════════════════════════════════════════════════════ */

void setup() {
  Serial.begin(115200);

  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2500) {
    delay(10);
  }

  Serial.println();
  Serial.println("GaitIMU v12 ESP32-C3 + MPU6050 node: " DEVICE_NAME);

  /* IMU bring-up */
  imuOk = imuInit();
  if (!imuOk) {
    Serial.println("[IMU] *** init failed — check wiring, I2C pins, address ***");
  }

  /* Fixed battery percentage unless you add real ADC battery support */
  batteryPct = 100;
  batteryLow = false;

  Serial.printf("[BAT] Battery stub: %u %%\n", batteryPct);

  /* BLE */
  BLEDevice::init(DEVICE_NAME);

  /*
     Important:
     Full 8-sample packets are 109 bytes.
     Default 23-byte MTU is too small.
  */
  BLEDevice::setMTU(247);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  pService = pServer->createService(BLEUUID(GAIT_SERVICE_UUID));

  cmdChar = pService->createCharacteristic(
    BLEUUID(CMD_CHAR_UUID),
    BLECharacteristic::PROPERTY_WRITE
  );

  timeChar = pService->createCharacteristic(
    BLEUUID(TIME_CHAR_UUID),
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  );

  dataChar = pService->createCharacteristic(
    BLEUUID(DATA_CHAR_UUID),
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );

  idChar = pService->createCharacteristic(
    BLEUUID(ID_CHAR_UUID),
    BLECharacteristic::PROPERTY_READ
  );

  /* CCC descriptors for notifications */
  BLE2902* timeDesc = new BLE2902();
  timeDesc->setAccessPermissions(ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE);
  timeChar->addDescriptor(timeDesc);

  BLE2902* dataDesc = new BLE2902();
  dataDesc->setAccessPermissions(ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE);
  dataChar->addDescriptor(dataDesc);

  cmdChar->setCallbacks(new CmdCallbacks());
  timeChar->setCallbacks(new TimeCallbacks());

  idChar->setValue((const uint8_t*)MODULE_ID, strlen(MODULE_ID));

  pService->start();

  /* Standard Battery Service stub */
  pBatteryService = pServer->createService(BLEUUID((uint16_t)0x180F));

  battChar = pBatteryService->createCharacteristic(
    BLEUUID((uint16_t)0x2A19),
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );

  BLE2902* battDesc = new BLE2902();
  battDesc->setAccessPermissions(ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE);
  battChar->addDescriptor(battDesc);

  battChar->setValue(&batteryPct, 1);
  pBatteryService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLEUUID(GAIT_SERVICE_UUID));
  pAdvertising->setScanResponse(true);

  BLEDevice::startAdvertising();

  Serial.println("Advertising as " DEVICE_NAME " — waiting for a connection…");
}

void loop() {
  /* Handle pending BLE commands */
  if (pendingStart) {
    pendingStart = false;

    if (!streaming) {
      packetFill = 0;
      packetSeq = 0;
      sampleSeq = 0;
      pktCount = 0;
      smpCount = 0;
      i2cErrors = 0;

      nextSampleMs = millis() + 2;
      streaming = true;

      Serial.println("[CMD] streaming started");
    }
  }

  if (pendingStop) {
    pendingStop = false;

    if (streaming) {
      streaming = false;
      flushPacket();

      Serial.println("[CMD] streaming stopped");
    }
  }

  if (pendingSetSpp) {
    pendingSetSpp = false;

    flushPacket();

    if (pendingSpp >= 1 && pendingSpp <= MAX_SAMPLES_PER_PACKET) {
      samplesPerPacket = pendingSpp;
    } else {
      samplesPerPacket = 1;
    }

    Serial.printf("[CMD] samples/packet = %u\n", samplesPerPacket);
  }

  /* Battery service update every 30 s */
  if (millis() - batLast > 30000) {
    batLast = millis();

    batteryPct = 100;
    batteryLow = false;

    if (bleConnected && battChar != nullptr) {
      battChar->setValue(&batteryPct, 1);
      battChar->notify();
    }
  }

  if (!bleConnected || !streaming) {
    delay(2);
    return;
  }

  /* 200 Hz sampling loop */
  uint32_t now = millis();

  /* If we fell far behind, re-anchor instead of bursting forever */
  if ((int32_t)(now - nextSampleMs) > 100) {
    nextSampleMs = now;
  }

  while ((int32_t)(nextSampleMs - now) <= 0) {
    addSample(nextSampleMs);
    nextSampleMs += SAMPLE_PERIOD_MS;
    now = millis();
  }

  /* Statistics */
  if (millis() - statLast >= 1000) {
    statLast = millis();

    if (Serial) {
      Serial.printf(
        "[STAT] samples=%u packets=%u seq=%u i2cErrors=%u\n",
        smpCount,
        pktCount,
        sampleSeq,
        i2cErrors
      );
    }
  }

  delay(1);
}