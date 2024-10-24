#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include <HCSR04.h>

// Player
SoftwareSerial DFPlayerSerial(10, 11); // RX, TX
DFRobotDFPlayerMini DFPlayer;
const unsigned long CHECK_INTERVAL = 2000;
const float STOP_STATUS = 0;
const int VOLUME = 14; // Set volume value. From 0 to 30
const int TRACK = 6;

// Ultrasonic 
const byte SONIC_TRIGGER = 5;
const byte SONIC_ECCO = 4;
UltraSonicDistanceSensor distanceSensor(SONIC_TRIGGER, SONIC_ECCO);

const float ERROR_VALUE = 1;
const float DETECT_DISTANCE = 30;

void setup()
{
    delay(1000);
    InitializePlayer();
}

void InitializePlayer()
{
    DFPlayerSerial.begin(9600);
    Serial.begin(115200);

    Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

    if (!DFPlayer.begin(DFPlayerSerial)) 
    {  //Use softwareSerial to communicate with mp3.
        Serial.println(F("Unable to begin:"));
        Serial.println(F("1.Please recheck the connection!"));
        Serial.println(F("2.Please insert the SD card!"));
        while(true)
        {
            delay(0); // Code to compatible with ESP8266 watch dog.
        }
    }
    DFPlayer.EQ(DFPLAYER_EQ_BASS);
    Serial.println(F("DFPlayer Mini online."));
}

void loop()
{
    if(!IsDetected()) return;
    // Serial.println("detected");

    Play();
}

bool IsDetected()
{
    float distance = distanceSensor.measureDistanceCm();
    bool isError = distance <= ERROR_VALUE;
    if(isError) return false;

    // Serial.println(distance); 
    return distance <= DETECT_DISTANCE ;
}

void Play()
{
    // Serial.println("Play");

    DFPlayer.volume(VOLUME); 
    DFPlayer.play(TRACK); 

    while (true) {
        if (!IsPlaying()) break;

        delay(CHECK_INTERVAL); 
    }
}

bool IsPlaying()
{
    int currentStatus = DFPlayer.readState();  // 현재 재생 상태를 읽어옴
    // Serial.println(currentStatus); 
    if (currentStatus == STOP_STATUS) return false;

    return true;  // 현재 재생 중
}