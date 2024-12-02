#define DEBUG
#define SOUND_SPEED 0.034
#define ERROR_VALUE 1
#define DETECT_DISTANCE 30

#include "ESP32Servo.h" 
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>

// Left Pin 
#define WIND_UP_PWM         15

#define SONIC_ECHO_LEFT     16
#define SONIC_TRIGGER_LEFT  17

#define MOTOR_ENA           5
#define MOTOR_IN1           18
#define MOTOR_IN2           19

// Right Pin
#define SONIC_ECHO_RIGHT    14
#define SONIC_TRIGGER_RIGHT 27

#define MOTOR_IN3           25
#define MOTOR_IN4           33
#define MOTOR_ENB           32

// Motor
const int MotorA[3] = { MOTOR_ENA, MOTOR_IN1, MOTOR_IN2 };
const int MotorB[3] = { MOTOR_ENB, MOTOR_IN3, MOTOR_IN4 };

const float LeftMultiple = 1.15;
const int MoveSpeed = 200;  
const int RotateSpeed = 170;
const int RotateDuration = 1000;

const char* BLUETOOTH_SENDER_NAMES[] = {"Cane", "Rock", "Sun"};
const int NUM_SENDERS = sizeof(BLUETOOTH_SENDER_NAMES) / sizeof(BLUETOOTH_SENDER_NAMES[0]);


Servo WindUp;
float RangeMin = 0.5;
float RangeMax = 2.5;

// Varialbles
int MoveDurationRange[2] = {3000, 5000}; 
int STOP_DURATION_WHEN_CANT_MOVE = 1000;

float LeftObstacleDistance;
bool ExistLeftObstacle;
float RightObstacleDistance;
bool ExistRightObstacle;

// Bluetooth Variables
int scanTime = 1;  // 스캔 주기 (초)
unsigned long lastResetTime = 0;        // 초기화 후 대기 시간 추적용
unsigned long prevScanTime = 0;
const long BLUETOOTH_INTERVAL = 1000;

void setup()
{
  InitSensor();
  InitMotorDriver();
  InitializeBluetooth();
}
void InitSensor()
{
  Serial.begin(115200);
  WindUp.attach(WIND_UP_PWM);

  // Ultrasonic 1
  pinMode(SONIC_TRIGGER_RIGHT, OUTPUT);
  pinMode(SONIC_ECHO_RIGHT, INPUT);

  // Ultrasonic 2
  pinMode(SONIC_TRIGGER_LEFT, OUTPUT);
  pinMode(SONIC_ECHO_LEFT, INPUT);
}
void InitMotorDriver()
{
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  pinMode(MOTOR_ENB, OUTPUT);
}
void InitializeBluetooth()
{
    BLEDevice::init("");
}
void loop() 
{
  // CarTest();
  // WindUpTest(); 
  MovementCycle();
}

void MovementCycle() 
{
  MoveForDuration(random(MoveDurationRange[0], MoveDurationRange[1]) );
  Stop(); 
  WaitForMoveDelay(); // Random duration 
}

void MoveForDuration(int duration) 
{
  Move();

  unsigned long startTime = millis();
  while (millis() - startTime < duration) 
  {
    if (!CanMove()) 
    {
      ChangeDirection();
      Move();
    }

    PlayWindUp();
  }
}
void Move()
{
  MoveMotorForward(MotorA, 20 * random(8, 12));
  MoveMotorForward(MotorB, 20 * random(8, 12));
}

bool CanMove()
{
  return !ExistObstacle() && IsInMoveRange();
}

bool ExistObstacle()
{
  // Initialize Variables
  ExistLeftObstacle = false;
  ExistRightObstacle = false;

  LeftObstacleDistance = MeasureDistance(SONIC_TRIGGER_LEFT, SONIC_ECHO_LEFT);
  ExistLeftObstacle = CheckObstacle(LeftObstacleDistance);

  RightObstacleDistance = MeasureDistance(SONIC_TRIGGER_RIGHT, SONIC_ECHO_RIGHT);
  ExistRightObstacle  = CheckObstacle(RightObstacleDistance);

#ifdef DEBUG
  String log = "Obstacle Left : " + String(ExistLeftObstacle) + " Right : " + String(ExistRightObstacle) + " Both : " + String(ExistLeftObstacle && ExistRightObstacle);
  Serial.println(log);  
#endif

  return ExistLeftObstacle || ExistRightObstacle;
}

float MeasureDistance(int tirgger, int echo)
{
    digitalWrite(tirgger, LOW);
    delayMicroseconds(2);
    digitalWrite(tirgger, HIGH);
    delayMicroseconds(10);
    digitalWrite(tirgger, LOW);

    // Measure the duration of the echo
    long duration = pulseIn(echo, HIGH, 30000); // Timeout: 30ms

    return duration * SOUND_SPEED / 2;
}

bool CheckObstacle(double distance)
{
  // Serial.println(distance);
  bool isError = distance <= ERROR_VALUE;
  if(isError) return false;

  // Serial.println(distance); 

  return distance <= DETECT_DISTANCE;
}

void ChangeDirection()
{
  Stop();
  WaitForRSSI();

  if(!IsInMoveRange())
  {
    TurnRight(180);
  }

  while (ExistObstacle())
  {
    if(ExistLeftObstacle && ExistRightObstacle)
    {
      TurnLeft(180);
    }

    if(ExistLeftObstacle)
    {
      TurnRight(10);  
    }
    else 
    {
      TurnLeft(10);
    }

    int stopDuration = random(500, 1000);
    delay(stopDuration);
  }
}
void WaitForRSSI() 
{

}

bool isTargetDevice(const String& deviceName) {
    for (int i = 0; i < NUM_SENDERS; i++) {
        if (deviceName == BLUETOOTH_SENDER_NAMES[i]) return true;
    }
    return false;
}

float CalculateDistance( int rssi) 
{
    return pow(10, (rssi - (-72)) / (-20.0));
}
void LoggingDistance(int foundDevice, int rssi, float distance) // , float filteredDistance
{
    Serial.print(foundDevice);
    Serial.print("수신된 RSSI: ");
    Serial.print(rssi);

    Serial.print(" | 추정 거리: ");
    Serial.print(distance);
    Serial.println(" m");

    // Serial.print(" | 필터된 거리: ");
    // Serial.print(filteredDistance);
    // Serial.println(" m");
}


void WaitForMoveDelay()
{
  // get random delay
  int stopDuration = random(1000, 5000);
  delay(stopDuration);
}

bool IsInMoveRange()
{
  unsigned long currentTime = millis();
  bool canScan = currentTime - prevScanTime >= BLUETOOTH_INTERVAL;
  if (!canScan) return true;

  prevScanTime = currentTime;

  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);

  BLEScanResults* foundDevices = pBLEScan->start(scanTime);  // scanTime만큼 스캔

  int foundBluetooth = 0;
  float distanceAmount = 0;

  for (int i = 0; i < foundDevices->getCount(); i++) 
  {
      BLEAdvertisedDevice device = foundDevices->getDevice(i);

      if (!isTargetDevice(device.getName())) continue;

      int rssi = device.getRSSI();
      float distance = CalculateDistance(rssi);

      foundBluetooth = foundBluetooth + 1;
      distanceAmount = distanceAmount + distance;
  }

  float resultDistance = distanceAmount / foundBluetooth ;

  LoggingDistance(foundBluetooth,0, resultDistance);

  pBLEScan->clearResults();

  return resultDistance >= RangeMin && resultDistance <= RangeMax || foundBluetooth == 0;
}

void Stop()
{
  StopCar();
  StopWindUp();
}


// Wind Up Methods
void PlayWindUp()
{
  WindUp.writeMicroseconds(1600);
  delay(30);
  WindUp.writeMicroseconds(1500);
  delay(100);
}
void StopWindUp()
{
  WindUp.writeMicroseconds(1500);
}
void WindUpTest()
{
  PlayWindUp();
  // StopWindUp();
  // delay(1000);
}

// Car Methods
void MoveCarForward() 
{
  MoveMotorForward(MotorA, (int)(MoveSpeed * LeftMultiple));
  MoveMotorForward(MotorB, MoveSpeed);
}
void MoveCarBackward()
{
  MoveMotorBackward(MotorA, (int)(MoveSpeed * LeftMultiple));
  MoveMotorBackward(MotorB, MoveSpeed);
}
void TurnLeft(float angle)
{
  float waitMultiple = angle / 180;
  MoveMotorForward(MotorA, (int)(RotateSpeed * LeftMultiple));
  MoveMotorBackward(MotorB , RotateSpeed);
  delay(RotateDuration * waitMultiple);
  StopCar();
}
void TurnRight(float angle)
{
  float waitMultiple = angle / 180;
  MoveMotorBackward(MotorA , (int)(RotateSpeed * LeftMultiple));
  MoveMotorForward(MotorB, RotateSpeed);
  delay(RotateDuration * waitMultiple);
  StopCar();
}
void StopCar()
{
  StopMotor(MotorA);
  StopMotor(MotorB);
}

// Motor Methods
void MoveMotorBackward(const int motor[3], int speed)
{
  digitalWrite(motor[1], HIGH);
  digitalWrite(motor[2], LOW);

  analogWrite(motor[0], speed);
}
void MoveMotorForward(const int motor[3], int speed)
{
  digitalWrite(motor[1], LOW);
  digitalWrite(motor[2], HIGH);

  analogWrite(motor[0], speed);
}
void StopMotor(const int motor[3])
{
  digitalWrite(motor[1], LOW);
  digitalWrite(motor[2], LOW);

  analogWrite(motor[0], 0);
}

// Test Methods
void CarTest()
{
  CarMoveTest();
  CarRotateTest();

  StopCar();
  delay(1000);
}
void CarMoveTest()
{
  Serial.println("Move Forward");
  MoveCarForward();
  delay(1000);
  StopCar();
  delay(2000);

  Serial.println("Move Backward");
  MoveCarBackward();
  delay(1000);
  StopCar();
  delay(2000);
}
void CarRotateTest()
{
  Serial.println("Rotate Left");
  TurnLeft(90);
  delay(3000);

  Serial.println("Rotate Right");
  TurnRight(90);
  delay(3000);
}