#include "KalmanFilter.h" 
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>

// Bluetooth Variables
int scanTime = 1;  // 스캔 주기 (초)
KalmanFilter kalman(0.1, 0.5, 1.0, 0);  // 칼만 필터 초기화
unsigned long lastResetTime = 0;        // 초기화 후 대기 시간 추적용
unsigned long prevScanTime = 0;
const long BLUETOOTH_INTERVAL = 100;
const char* BLUETOOTH_SENDER_NAME = "Cane";

void setup()
{
    Serial.begin(115200);
    InitializeBluetooth();
}
void InitializeBluetooth()
{
    BLEDevice::init("");
}

void loop() 
{
    CheckBluetoothDistance();
}

void CheckBluetoothDistance()
{
    unsigned long currentTime = millis();
    bool canScan = currentTime - prevScanTime >= BLUETOOTH_INTERVAL;
    if (!canScan) return;

    prevScanTime = currentTime;

    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setActiveScan(true);

    // 포인터로 스캔 결과를 받기
    BLEScanResults* foundDevices = pBLEScan->start(scanTime);  // scanTime만큼 스캔

    // 스캔된 각 장치에 대해 처리
    for (int i = 0; i < foundDevices->getCount(); i++) {
        BLEAdvertisedDevice device = foundDevices->getDevice(i);
        
        if (device.getName() == BLUETOOTH_SENDER_NAME) 
        {
            int rssi = device.getRSSI();
            // float distance = GetDistanceFromRSSI(rssi);
            float distance = pow(10, (rssi - (-72)) / (-20.0));

            // float filteredDistance = kalman.updateEstimate(distance);

            LoggingDistance(rssi, distance);
        }
    }

    // 스캔 결과 메모리 해제
    pBLEScan->clearResults();

}
float GetDistanceFromRSSI(int rssi)
{
    pow(10, (rssi - (-69)) / (-20.0));
}

void LoggingDistance(int rssi, float distance) // , float filteredDistance
{
    Serial.print("수신된 RSSI: ");
    Serial.print(rssi);

    Serial.print(" | 추정 거리: ");
    Serial.print(distance);
    Serial.println(" m");

    // Serial.print(" | 필터된 거리: ");
    // Serial.print(filteredDistance);
    // Serial.println(" m");
}
