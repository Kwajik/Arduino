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

const float LeftMultiple = 1.15;
const int MoveSpeed = 200;  
const int RotateSpeed = 170;
const int RotateDuration = 1000;

// Ultrasonic Sensor
const float ERROR_VALUE = 1;
const float DETECT_DISTANCE = 30;
const byte ECHO_COUNT = 2;
const byte* SONIC_ECHO_PINS = new byte[ECHO_COUNT] { SONIC_ECHO_PIN_1, SONIC_ECHO_PIN_2};

// Wind Up Motor
Servo WindUpMotor;

// Varialbles
float Angle = 0;
int MoveDurationRange[2] = {3000, 5000}; 
int STOP_DURATION_WHEN_CANT_MOVE = 1000;

float LeftObstacleDistance;
bool ExistLeftObstacle;
float RightObstacleDistance;
bool ExistRightObstacle;

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
  // CarTest();
  MovementCycle();
}

void MovementCycle() 
{
  MoveForDuration(random(MoveDurationRange[0], MoveDurationRange[1]) );
  Stop(); 
  WaitForMoveDelay(); // Random duration 
}

void MoveForDuration(int duration) 
{x
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

  double* distances = HCSR04.measureDistanceCm();

  LeftObstacleDistance = distances[0];
  ExistLeftObstacle = CheckObstacle(LeftObstacleDistance);

  RightObstacleDistance = distances[1];
  ExistRightObstacle  = CheckObstacle(RightObstacleDistance);

#ifdef DEBUG
  String log = "Obstacle Left : " + String(ExistLeftObstacle) + " Right : " + String(ExistRightObstacle);
  Serial.println(log);  
#endif

  return ExistLeftObstacle || ExistRightObstacle;
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

void WaitForMoveDelay()
{
  // get random delay
  int stopDuration = random(1000, 5000);
  delay(stopDuration);
}

bool IsInMoveRange()
{
  return true;
  // check distance from RSSI
}

void Stop()
{
  StopCar();
  StopWindUp();
}


// Wind Up Methods
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