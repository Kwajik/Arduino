#include <HCSR04.h>

const byte triggerPin = 5;
const byte echoPin = 4;
const float ERROR_VALUE = 1;
const float DETECT_DISTANCE = 30;

UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);

int buzzer = 13;

void setup () {
  pinMode(buzzer,OUTPUT);
  Serial.begin(9600);
}

void loop () {
  if(IsDetected()){
    // Serial.println("detected");
    // tone(buzzer,262,500);
    delay(500);      
  }    
}

bool IsDetected(){
  float distance = distanceSensor.measureDistanceCm();
  bool isError = distance <= ERROR_VALUE;
  if(isError) return false;

  Serial.println(distance); 
  return distance <= DETECT_DISTANCE ;
}