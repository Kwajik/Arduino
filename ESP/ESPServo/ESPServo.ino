#include "ESP32Servo.h" 
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define LEFT_PIN 27 
#define RIGHT_PIN 14 
#define STOP 1500

Servo Left; 
Servo Right; 

unsigned long MoveTime;
unsigned long StopTime;
unsigned long BluetoothTime;
// Bluetooth
BLEServer *pServer = NULL;
BLECharacteristic *Sender = NULL;

const char* BLUETOOTH_SENDER_NAME = "Rock";

const long BLUETOOTH_INTERVAL = 1000; // 1초 간격

// 광고 간격 상수 정의
const uint16_t MIN_INTERVAL = 0x20;
const uint16_t MAX_INTERVAL = 0x30;

// UUID 상수 정의
const BLEUUID SERVICE_UUID((uint16_t)0x1823); 
const BLEUUID CHARACTERISTIC_UUID((uint16_t)0x2A56); 

void setup() 
{ 
    Left.attach(LEFT_PIN);
    Right.attach(RIGHT_PIN);
    InitializeBluetooth();

    MoveTime = millis();
    RandomMove();
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
    Move();
    SendBluetoothData();
    // Backward(1000);
    // Stop(3000);
}

// void Forward(int duration)
// {
//   Left.writeMicroseconds(1600);
//   Right.writeMicroseconds(1350);
//   delay(duration);
// }
// void Backward(int duration)
// {
//   Left.writeMicroseconds(1470 - 145);
//   Right.writeMicroseconds(1470 + 45);
//   delay(duration);
// }


void Move()
{
  unsigned long currentTime = millis();
  bool canMove = currentTime - MoveTime < 1000;
  if(canMove)
  {
    StopTime = currentTime;
    Serial.println("Move");
  }
  else
  {
    bool canStop = currentTime - StopTime < 3000;
    if(canStop)
    {
      Serial.println("Stop");
      Left.writeMicroseconds(STOP);
      Right.writeMicroseconds(STOP);
    }
    else
    {
      Serial.println("Change");
      MoveTime = currentTime;
      RandomMove();
    }
  }
}

void RandomMove()
{
  Left.writeMicroseconds(random(1200, 1800));
  Right.writeMicroseconds(random(1200, 1800));
}
void Stop()
{
  Left.writeMicroseconds(STOP);
  Right.writeMicroseconds(STOP);
}

void SendBluetoothData()
{
    unsigned long currentTime = millis();
    bool canSend = currentTime - BluetoothTime >= BLUETOOTH_INTERVAL;
    if (!canSend) return;

    BluetoothTime = currentTime;
    Sender->notify();
    Serial.println("데이터 전송 중...");    
}