#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// Bluetooth
BLEServer *pServer = NULL;
BLECharacteristic *Sender = NULL;

const char* BLUETOOTH_SENDER_NAME = "ESP32_Sender";

unsigned long prevTime = 0;
const long BLUETOOTH_INTERVAL = 1000; // 1초 간격

// 광고 간격 상수 정의
const uint16_t MIN_INTERVAL = 0x20;
const uint16_t MAX_INTERVAL = 0x30;

// UUID 상수 정의
const BLEUUID SERVICE_UUID((uint16_t)0x180F);
const BLEUUID CHARACTERISTIC_UUID((uint16_t)0x2A19);

void setup() 
{
    Serial.begin(115200);
    InitializeBluetooth();
}

void InitializeBluetooth()
{
    BLEDevice::init(BLUETOOTH_SENDER_NAME);  // 디바이스 이름 설정
    pServer = BLEDevice::createServer();
    
    BLEService *pService = pServer->createService(SERVICE_UUID);  // 서비스 UUID 사용
    Sender = pService->createCharacteristic(
                        CHARACTERISTIC_UUID,  // 특성 UUID 사용
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_NOTIFY
                      );

    Sender->setValue("Hello Receiver!");  // 초기 값
    pService->start();

    // 광고 설정
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);  // 서비스 UUID 사용
    pAdvertising->setScanResponse(true);

    pAdvertising->setMinInterval(MIN_INTERVAL);    // 최소 광고 간격
    pAdvertising->setMaxInterval(MAX_INTERVAL);    // 최대 광고 간격

    BLEDevice::startAdvertising();
    Serial.println("Bluetooth Initialized");
}

void loop() 
{
    SendBluetoothData();
}

void SendBluetoothData()
{
    unsigned long currentTime = millis();
    bool canSend = currentTime - prevTime >= BLUETOOTH_INTERVAL;
    if (!canSend) return;

    prevTime = currentTime;
    Sender->notify();
    Serial.println("데이터 전송 중...");    
}