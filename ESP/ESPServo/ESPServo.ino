#include "ESP32Servo.h" 
#define SERVO_PIN 15 
Servo myservo; 
void setup() 
{ 
    myservo.attach(15);
}
      
void loop() 
{ 
  myservo.writeMicroseconds(1600);
  delay(50);
  myservo.writeMicroseconds(1500);
  delay(100);
}