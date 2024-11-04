#include <HCSR04.h>

const int pinENA = 5;
const int pinIN1 = 6;
const int pinIN2 = 7;
const int pinIN3 = 8;
const int pinIN4 = 9;
const int pinENB = 10;

const int STOP_DELAY = 2000;  //espera entre fases
const int waitTime = 2000;  //espera entre fases
const int speed = 200;    //velocidad de giro

const int pinMotorA[3] = { pinENA, pinIN1, pinIN2 };
const int pinMotorB[3] = { pinENB, pinIN3, pinIN4 };

// Ultrasonic 
const byte SONIC_TRIGGER_PIN = 2;
const byte ECHO_COUNT = 2;
const byte* SONIC_ECHO_PINS = new byte[ECHO_COUNT] { 3, 4 };

const float ERROR_VALUE = 1;
const float DETECT_DISTANCE = 10;

void setup()
{
  Serial.begin(115200);
  HCSR04.begin(SONIC_TRIGGER_PIN, SONIC_ECHO_PINS, ECHO_COUNT);
  InitMotorDriver();
}

void InitMotorDriver()
{
  pinMode(pinIN1, OUTPUT);
  pinMode(pinIN2, OUTPUT);
  pinMode(pinENA, OUTPUT);
  pinMode(pinIN3, OUTPUT);
  pinMode(pinIN4, OUTPUT);
  pinMode(pinENB, OUTPUT);
}

void loop()
{
  if(CanMove())
  {
    return;
  }
  else
  {
    return;
    Stop();
    delay(STOP_DELAY);
  }

  // if time out -> stop 
  // Calc random move chance 

  MoveTest();
}

bool CanMove()
{
  if(IsOutOfMovableBoundary()) return false;
  if(ExistObstacle()) return false;
  if(IsMoveTimeOver()) return false;

  return true;
}
// 진행 방향만 체크 
bool ExistObstacle()
{
  double* distances = HCSR04.measureDistanceCm();

  bool existFront = CheckObstacle(distances[0]);
  bool existBack  = CheckObstacle(distances[1]);

  return existFront || existBack;
}
bool CheckObstacle(double distance)
{
  Serial.println(distance);
  bool isError = distance <= ERROR_VALUE;
  if(isError) return false;

  bool existObstacle = distance <= DETECT_DISTANCE;

  // Use Debug
  // Serial.println(distance); 
  if(existObstacle)
  {
    Serial.println("Detect Obstacle ");   
  }

  return existObstacle;
}

bool IsMoveTimeOver()
{
  return false;
}

bool IsOutOfMovableBoundary()
{
  return false;
  // check distance from another arduino bluetooth
}



void Move()
{
  MoveCar();
  MoveWindUp();
}

void Stop()
{
  StopCar();
  StopWindUp();
}

// Car Movement
void MoveCar()
{

}

void moveForward(const int pinMotor[3], int speed)
{
  digitalWrite(pinMotor[1], HIGH);
  digitalWrite(pinMotor[2], LOW);

  analogWrite(pinMotor[0], speed);
}

void moveBackward(const int pinMotor[3], int speed)
{
  digitalWrite(pinMotor[1], LOW);
  digitalWrite(pinMotor[2], HIGH);

  analogWrite(pinMotor[0], speed);
}

void fullStop(const int pinMotor[3])
{
  digitalWrite(pinMotor[1], LOW);
  digitalWrite(pinMotor[2], LOW);

  analogWrite(pinMotor[0], 0);
}

void StopCar()
{

}

void MoveWindUp()
{

}

void StopWindUp()
{

}

void MoveTest()
{
  moveForward(pinMotorA, 100);
  moveForward(pinMotorB, 100);
  delay(waitTime);

  moveBackward(pinMotorA, 10);
  moveBackward(pinMotorB, 10);
  delay(waitTime);

  fullStop(pinMotorA);
  fullStop(pinMotorB);
  delay(waitTime);
}