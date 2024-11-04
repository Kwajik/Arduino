#include <Servo.h>

Servo myservo;
// 0 clockwise
// 90 ~ 96 Stop
// 180 Counter clockwise

void setup() {
  myservo.attach(2);
}

void loop() 
{
  Play();
}
void Play()
{
  // myservo.writeMicroseconds(1534);
  myservo.writeMicroseconds(2500);
  // delay(10);
  delay(50);
  myservo.writeMicroseconds(1500);
  // delay(70);
  delay(100);
}