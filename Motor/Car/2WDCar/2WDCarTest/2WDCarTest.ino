#include <HCSR04.h>
#include <Servo.h>
#define DEBUG 

// Pin
const byte WIND_UP_PWM_PIN   = 2;
const byte SONIC_TRIGGER_PIN = 3;
const byte SONIC_ECHO_PIN_1  = 4;
const byte SONIC_ECHO_PIN_2  = 5;
const byte MOTOR_ENA         = 6;
const byte MOTOR_IN1         = 7;
const byte MOTOR_IN2         = 8;
const byte MOTOR_IN3         = 9;
const byte MOTOR_IN4         = 10;
const byte MOTOR_ENB         = 11;

// Motor
const int MotorA[3] = { MOTOR_ENA, MOTOR_IN1, MOTOR_IN2 };
const int MotorB[3] = { MOTOR_ENB, MOTOR_IN3, MOTOR_IN4 };

const int speed = 200;    //velocidad de giro

// Ultrasonic Sensor
const float ERROR_VALUE = 1;
const float DETECT_DISTANCE = 10;
const byte ECHO_COUNT = 2;
const byte* SONIC_ECHO_PINS = new byte[ECHO_COUNT] { SONIC_ECHO_PIN_1, SONIC_ECHO_PIN_2};

// Wind Up Motor
Servo WindUpMotor;

// Varialbles
int MoveDurationRange[2] = {3000, 5000}; 
int STOP_DURATION_WHEN_CANT_MOVE = 1000;

bool ExistFrontObstacle;
bool ExistBackObstacle;

// Initialize
void setup()
{
  InitSensor();
  InitMotorDriver();
}

void InitSensor()
{
  Serial.begin(115200);
  HCSR04.begin(SONIC_TRIGGER_PIN, SONIC_ECHO_PINS, ECHO_COUNT);
  WindUpMotor.attach(WIND_UP_PWM_PIN);
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

// Movememt
void loop()
{
  ChangeDirection();

  unsigned long startTime = millis();
  int moveDuration = MoveDurationRange[1];
  while (millis() - startTime < moveDuration)
  {
    if(!CanMove()) 
    {
      Stop();
      delay(STOP_DURATION_WHEN_CANT_MOVE);

      ChangeDirection();
    }

    PlayWindUp();
  }

  Stop();
  WaitForMoveDelay();
}

void ChangeDirection()
{
  moveForward(MotorA, 100);
  moveForward(MotorB, 100);

  // 진행 방향의 장애물이 생기면 멈추고 반대 방향으로 움직인다. 
  // 그러다가 원래 방향으로 움직임이 가능해지면 다시 움직인다. 
  // 앞 뒤로 방해물이 있다면 회전하면서 방해물이 없는 쪽을 찾음 
}

void WaitForMoveDelay()
{
  // get random delay
  int stopDuration = 10000;
  delay(stopDuration);
}

// Check logic
bool CanMove()
{
  if(ExistObstacle()) return false;
  if(IsOutOfMovableBoundary()) return false;

  return true;
}

bool ExistObstacle()
{
  double* distances = HCSR04.measureDistanceCm();

  ExistFrontObstacle = CheckObstacle(distances[0]);
  ExistBackObstacle  = CheckObstacle(distances[1]);

#ifdef DEBUG
  String log = "Obstacle Front : " + String(ExistFrontObstacle) + " Back : " + String(ExistBackObstacle);
  Serial.println(log);  
#endif

  return ExistFrontObstacle || ExistBackObstacle;
}

bool CheckObstacle(double distance)
{
  // Serial.println(distance);
  bool isError = distance <= ERROR_VALUE;
  if(isError) return false;

  // Serial.println(distance); 

  return distance <= DETECT_DISTANCE;
}

bool IsOutOfMovableBoundary()
{
  return false;
  // check distance from another arduino bluetooth
}

void Stop()
{
  StopCar();
  StopWindUp();
}

void StopCar()
{
  fullStop(MotorA);
  fullStop(MotorB);
}

void PlayWindUp()
{
  WindUpMotor.writeMicroseconds(2500);
  delay(30);
  WindUpMotor.writeMicroseconds(1500);
  delay(100);
}

void StopWindUp()
{
  WindUpMotor.writeMicroseconds(1500);
}

void MoveTest()
{
  moveForward(MotorA, 100);
  moveForward(MotorB, 100);
  delay(2000);

  moveBackward(MotorA, 10);
  moveBackward(MotorB, 10);
  delay(2000);

  fullStop(MotorA);
  fullStop(MotorB);
  delay(2000);
}

void moveForward(const int motor[3], int speed)
{
  digitalWrite(motor[1], HIGH);
  digitalWrite(motor[2], LOW);

  analogWrite(motor[0], speed);
}

void moveBackward(const int motor[3], int speed)
{
  digitalWrite(motor[1], LOW);
  digitalWrite(motor[2], HIGH);

  analogWrite(motor[0], speed);
}

void fullStop(const int motor[3])
{
  digitalWrite(motor[1], LOW);
  digitalWrite(motor[2], LOW);

  analogWrite(motor[0], 0);
}