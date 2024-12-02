#include "HardwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// Pin
#define DF_RX 26
#define DF_TX 25
#define Ultrasonic_Trigger 14
#define Ultrasonic_ECHO 27

// Setting
#define DETECT_DISTANCE 30
#define SOUND_SPEED 0.034
#define CHECK_INTERVAL 2000
#define VOLUME 14

unsigned long Mp3Time;
unsigned long BluetoothTime;

HardwareSerial DFPlayerSerial(1); // UART1 사용 (GPIO33 = RX, GPIO32 = TX)
DFRobotDFPlayerMini DFPlayer;

// Bluetooth
BLEServer *pServer = NULL;
BLECharacteristic *Sender = NULL;

const char* BLUETOOTH_SENDER_NAME = "Rock";

const long BLUETOOTH_INTERVAL = 1000; // 1초 간격

// 광고 간격 상수 정의
const uint16_t MIN_INTERVAL = 0x20;
const uint16_t MAX_INTERVAL = 0x30;

// UUID 상수 정의
const BLEUUID SERVICE_UUID((uint16_t)0x181C); // 사용자 정의 Service UUID 예제
const BLEUUID CHARACTERISTIC_UUID((uint16_t)0x2A3D); // 사용자 정의 Characteristic UUID 예제

void setup() 
{
    delay(1000);

    Serial.begin(115200);

    IinitializeUltrasonic();
    InitializeMP3();
    InitializeBluetooth();
}

void IinitializeUltrasonic()
{
    pinMode(Ultrasonic_Trigger, OUTPUT); // Sets the trigPin as an Output
    pinMode(Ultrasonic_ECHO, INPUT); // Sets the echoPin as an Input
}
void InitializeMP3()
{
    DFPlayerSerial.begin(9600, SERIAL_8N1, DF_RX, DF_TX);

    Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

    if (!DFPlayer.begin(DFPlayerSerial)) 
    { 
        Serial.print(F("Unable to begin:"));
        uint8_t errorType = DFPlayer.readType();  // 에러 타입 읽기
        switch (errorType) {
            case TimeOut: 
                Serial.println(F("Connection timeout!"));
                break;
            case WrongStack: 
                Serial.println(F("Wrong stack!"));
                break;
            case DFPlayerCardInserted: 
                Serial.println(F("SD Card not detected!"));
                break;
            default: 
                Serial.println(F("Unknown error!"));
                break;
        }
        while (true) {
            delay(0); // Watchdog 방지용
        }
    }
    DFPlayer.EQ(DFPLAYER_EQ_BASS);
    Serial.println(F("DFPlayer Mini online."));
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
    PlayMP3();
    SendBluetoothData();
}

bool IsDetected()
{
    digitalWrite(Ultrasonic_Trigger, LOW);
    delayMicroseconds(2);
    digitalWrite(Ultrasonic_Trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(Ultrasonic_Trigger, LOW);

    // Measure the duration of the echo
    long duration = pulseIn(Ultrasonic_ECHO, HIGH, 30000); // Timeout: 30ms

    if (duration == 0) 
    {
        // Serial.println("No echo received. Check your wiring or adjust the sensor position.");
        return false;
    }
    else 
    {
        float distance = duration * SOUND_SPEED / 2;

        // Serial.print("Distance (cm): ");
        // Serial.println(distance);
        return distance <= DETECT_DISTANCE ;
    }
}

bool MP3Play;
void PlayMP3()
{
    if(MP3Play)
    {
        unsigned long currentTime = millis();
        bool canCheck = currentTime - Mp3Time >= CHECK_INTERVAL;
        if (canCheck)
        {
            Mp3Time = currentTime;
            if (IsPlaying()) return;

          MP3Play = false;
        }
    }
    else
    {
        if(!IsDetected()) return;

        Serial.println("detected");
        MP3Play = true;
        Mp3Time = millis();
        DFPlayer.volume(VOLUME); 
        DFPlayer.play(1); 
    }
}

const float STOP_STATUS = 0;
bool IsPlaying()
{
    int currentStatus = DFPlayer.readState();  // 현재 재생 상태를 읽어옴
    // Serial.println(currentStatus); 
    if (currentStatus == STOP_STATUS) return false;

    return true;  // 현재 재생 중
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