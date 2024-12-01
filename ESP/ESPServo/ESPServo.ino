#include "ESP32Servo.h" 

#define LEFT_PIN 13 
#define RIGHT_PIN 12 
#define STOP 1500

Servo Left; 
Servo Right; 

void setup() 
{ 
    Left.attach(LEFT_PIN);
    Right.attach(RIGHT_PIN);
}
      
void loop() 
{ 
    Forward(1000);
    Stop(3000);
    // Backward(1000);
    // Stop(3000);
}
void Forward(int duration)
{
  Left.writeMicroseconds(1500 + 85);
  Right.writeMicroseconds(1500 - 75);
  delay(duration);
}
void Backward(int duration)
{
  Left.writeMicroseconds(1500 - 145);
  Right.writeMicroseconds(1500 + 45);
  delay(duration);
}
void Stop(float duration)
{
  Left.writeMicroseconds(STOP);
  Right.writeMicroseconds(STOP);
  delay(duration);
}