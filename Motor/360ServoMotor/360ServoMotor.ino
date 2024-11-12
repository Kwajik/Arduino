#include <Servo.h>

Servo myservo;

void setup() {
  myservo.attach(2);
}

void loop() 
{
  Play();
}
void Play()
{
  myservo.writeMicroseconds(2500);
  delay(50);
  myservo.writeMicroseconds(1500);
  delay(100);
}