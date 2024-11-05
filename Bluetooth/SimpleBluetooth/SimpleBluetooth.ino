#include <SoftwareSerial.h>           // 소프트웨어 시리얼 라이브러리 불러오기

const byte BLUETOOTH_RX = 2;
const byte BLUETOOTH_TX = 3;

SoftwareSerial BTSerial(BLUETOOTH_TX, BLUETOOTH_RX);        // 소프트웨어 시리얼 객체를 3(TX),4번(RX) 으로 생성

void setup()
{
  Serial.begin(9600);                // 시리얼 통신을 개시, 속도는 9600  
  BTSerial.begin(9600);              // 소프트웨어시리얼 통신 개시, 속도는 9600
}

void loop()
{
  if(Serial.available())              // 시리얼 통신으로 문자가 들어오면
  {
    BTSerial.write(Serial.read());      // 블루투스시리얼 통신으로 발송
  }
  
  if(BTSerial.available())               // 블루투스 시리얼 통신으로 문자가 들어오면
  {
    Serial.write(BTSerial.read());       // 시리얼 창에 표시(시리얼 통신으로 출력)
  }
}