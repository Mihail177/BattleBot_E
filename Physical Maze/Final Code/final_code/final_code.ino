#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>

boolean hasInitiatedStart = false;
boolean hasStarted = false;

//===========[ LineSensors ]================

const int lineSensors[] = {A0, A1, A2, A3, A4,A5, A6, A7};
int lineSensorSensitivity = 800;

//============[ Gripper ]======================
const int GRIPPER_PIN=6;
const int GRIPPER_OPEN_PULSE=1600;
const int GRIPPER_CLOSE_PULSE=970;
const int GRIPPER_PULSE_REPEAT=10;

//============[ Time variables ]================
unsigned long time;

//===========[ DistanceSensors ]================

long durationFront;
int distanceFront;
const int trigPinFront = 8;
const int echoPinFront = 12;

long durationLeft;
int distanceLeft;
const int trigPinLeft = 4;
const int echoPinLeft = 7;

//=========[ Motor pins ]=============

const int motorRightBackwards=10;
const int motorRightForward=9;  
const int motorLeftBackwards=5;
const int motorLeftForward=11;
const int movementStuckBufferDelay=1700;

//==========[ Motor tests for pulses ]========

const int motor_R1=2;
const int motor_R2=3;
int countLeft=0;
int countRight=0;
int countsLeft=0,previousCountLeft;
int countsRight=0,previousCountRight;


//===========[ Led Pixels ]====================

const int PIXEL_PIN=13;
const int PIXEL_NUMBER=4;
Adafruit_NeoPixel leds(PIXEL_NUMBER, PIXEL_PIN, NEO_RGB + NEO_KHZ800);
const uint32_t RED=leds.Color(255,0,0);
const uint32_t YELLOW=leds.Color(255,150,0);
const uint32_t BLUE=leds.Color(0,0,255);
const uint32_t WHITE=leds.Color(255,255,255);
const uint32_t START=leds.Color(0,0,0);

//===============[ SETUP ]=================

void setup() {
  pinMode(motor_R1, INPUT);
  pinMode(motor_R2, INPUT);
  attachInterrupt(digitalPinToInterrupt(motor_R1),CountA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor_R2),CountB,CHANGE);
  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
  pinMode(trigPinLeft, OUTPUT); // Sets the left trigPin as an Output
  pinMode(echoPinLeft, INPUT); // Sets the left echoPin as an Input
  setupLineSensors();
  Serial.begin(9600);
  leds.begin();
  leds.fill(BLUE,0,4);
  leds.show();
  countLeft=0;
  countRight=0; 
}

//=========[ LOOP ]=================

void loop()
{
  if(hasStarted)
  {
   closeGripper();
   getDistanceLeft();
   getDistanceFront();
   //In case that there is a line, follow line
   followLine();
   if(isBlackZone())
   {
    stopRobot();
    openGripper();
    wait(300);
    hasStarted = false;
    hasInitiatedStart = false;
   }
   if (distanceLeft<=17&&distanceFront>=12)
   {
    moveForwardOnPulses(10);
    wait(100);
   }
   else if(distanceLeft>17 && distanceFront>12)
    {
      turnLeftOnPulses(36);
      wait(200);
      moveForwardOnPulses(15);
      turnLeftOnPulses(36);
    }
    else
    {
      leds.fill(YELLOW,0,4);
      leds.show();
      moveBackwards();
      wait(100);
      stopRobot();
      while(distanceFront<15)
      {
      rotateCounterAxis();
      getDistanceFront();
      }    
    }
  }
  else 
  {
    if(hasInitiatedStart == true)
    {
      moveForward();
      wait(750);
      closeGripper();
      countLeft=0;
      countRight=0;
      turnLeftOnPulses(36);
      moveForwardOnPulses(70);
      hasStarted = true;
    }
    else 
    {
      wait(1000);
      getDistanceFront();
      if(distanceFront < 30)
      {
        openGripper();
        wait(1500);
        hasInitiatedStart = true;
      }
    }
  }
}

//========[ Functions ]=================

void wait(int waitingTime)
{
  time = millis();
  while(millis() < time + waitingTime)
  {
  }
}

//Opening and closing the gripper based on pulses
void gripperServo(int pulse)
{
    for(int i = 0; i < GRIPPER_PULSE_REPEAT; i++)
    {
        digitalWrite(GRIPPER_PIN,HIGH);
        delayMicroseconds(pulse);
        digitalWrite(GRIPPER_PIN,LOW);
        delay(20);
    }
}

void openGripper()
{
    gripperServo(GRIPPER_OPEN_PULSE);
}
void closeGripper()
{
  gripperServo(GRIPPER_CLOSE_PULSE);
}



boolean isBlackZone()
{
  int count = 0;
  for(int i = 0; i < 8; i++)
  {
    if(analogRead(lineSensors[i]) > lineSensorSensitivity)
    {
      count ++;
    }
  }
  if(count < 6)
  {
    return false;
  }
  return true;
}

void setupLineSensors()
{
 pinMode(A0, INPUT);
 pinMode(A1, INPUT);
 pinMode(A2, INPUT);
 pinMode(A3, INPUT);
 pinMode(A4, INPUT);
 pinMode(A6, INPUT);
 pinMode(A7, INPUT);
}

// Logic for follow line
void followLine()
{
  if(analogRead(lineSensors[2]) > lineSensorSensitivity && analogRead(lineSensors[3]) > lineSensorSensitivity)
  {
    analogWrite(motorLeftForward, 255);
    analogWrite(motorRightForward, 140);
  }
  else if(analogRead(lineSensors[4]) > lineSensorSensitivity && analogRead(lineSensors[5]) > lineSensorSensitivity)
  {
    analogWrite(motorLeftForward, 140);
    analogWrite(motorRightForward, 255);
  }
  else if(analogRead(lineSensors[1]) > lineSensorSensitivity && analogRead(lineSensors[2]) > lineSensorSensitivity)
  {
    analogWrite(motorLeftForward, 255);
    analogWrite(motorRightForward, 0);
  }
  else if(analogRead(lineSensors[5]) > lineSensorSensitivity && analogRead(lineSensors[6]) > lineSensorSensitivity)
  {
    analogWrite(motorLeftForward, 0);
    analogWrite(motorRightForward, 255);
  }
  else if(analogRead(lineSensors[1]) > lineSensorSensitivity && analogRead(lineSensors[0]) > lineSensorSensitivity)
  {
    analogWrite(motorLeftForward, 255);
    analogWrite(motorRightForward, 0);
  }
  else if(analogRead(lineSensors[6]) > lineSensorSensitivity && analogRead(lineSensors[7]) > lineSensorSensitivity)
  {
    analogWrite(motorLeftForward, 0);
    analogWrite(motorRightForward, 255);
  }

}

void setupMotorPins()
{ 
  pinMode(motorRightBackwards,OUTPUT);
  pinMode(motorRightForward,OUTPUT);
  pinMode(motorLeftBackwards,OUTPUT);
  pinMode(motorLeftForward,OUTPUT);
}
void moveForward()
{
  leds.fill(BLUE,0,4);
  leds.show();
  analogWrite(motorLeftForward,230);
  analogWrite(motorRightForward,255);
  analogWrite(motorRightBackwards,0);
  analogWrite(motorLeftBackwards,0);
}

void moveBackwardsRotate()
{
  analogWrite(motorRightBackwards,255);
  analogWrite(motorLeftBackwards,110);
  analogWrite(motorRightForward, 0);
  analogWrite(motorLeftForward,0);
}

void moveBackwards()
{
  analogWrite(motorRightBackwards,255);
  analogWrite(motorLeftBackwards, 255);
  analogWrite(motorRightForward, 0);
  analogWrite(motorLeftForward,0);
}

void stopRobot() 
{
  analogWrite(motorRightBackwards,0);
  analogWrite(motorRightForward, 0);
  analogWrite(motorLeftBackwards, 0);
  analogWrite(motorLeftForward,0);
}
void turnLeft()
{
  leds.fill(RED, 0, 4);
  leds.show();
  analogWrite(motorLeftBackwards,0);
  analogWrite(motorLeftForward, 0);
  analogWrite(motorRightForward, 200);
  analogWrite(motorRightBackwards,0);
}
void adjustLeft()
{
  analogWrite(motorLeftBackwards,0);
  analogWrite(motorLeftForward, 255);
  analogWrite(motorRightForward, 210);
  analogWrite(motorRightBackwards,0);
}
void adjustRight()
{
  analogWrite(motorLeftBackwards,0);
  analogWrite(motorLeftForward, 150);
  analogWrite(motorRightForward, 255);
  analogWrite(motorRightBackwards,0);
}
void rotateCounterAxis()
{
  analogWrite(motorLeftBackwards,0);
  analogWrite(motorLeftForward, 220);
  analogWrite(motorRightForward, 0);
  analogWrite(motorRightBackwards,220);
}
void turnLeftOnPulses(int nrOfPulses)
{
  stopRobot();
  countLeft=0;
  countRight=0;
  int lastCountRight = -1;
  int lastCountLeft = -1;
  long movementBuffer = millis() + movementStuckBufferDelay;
  boolean isActive = true;
  getDistanceLeft();
  turnLeft();
  while ((countLeft<nrOfPulses && countRight<nrOfPulses) && distanceLeft>=10 && isActive)
    {
      if(countLeft == lastCountLeft && countRight == lastCountRight)
      { //wheel has not pulsed yet
        if(millis() > movementBuffer && distanceLeft > 10)
        { //if the robot not moved for duration
          movementBuffer = millis() + movementStuckBufferDelay;
          leds.fill(WHITE, 0, 4);
          leds.show();
          moveBackwardsRotate();
          wait(400);
          stopRobot();
        }
      }
      else
      {
        movementBuffer = millis() + movementStuckBufferDelay;
        lastCountLeft = countLeft;
        lastCountRight = countRight;
      }
      getDistanceLeft();
      getDistanceFront();
      }
  stopRobot();
}

void moveForwardOnPulses(int nrOfPulses)
{
  stopRobot();
  countLeft=0;
  countRight=0;
  int lastCountRight = -1;
  int lastCountLeft = -1;
  long movementBuffer = millis() + movementStuckBufferDelay;
  boolean isActive = true;
  if(distanceLeft<=5)
  {
    adjustLeft();
  }
  else if(distanceLeft>=13 && distanceLeft<20)
  {
    adjustRight();
  }
  else
  {
    moveForward();
  }
  while ((countLeft<nrOfPulses && countRight<nrOfPulses) && isActive)
  {
    if(countLeft == lastCountLeft && countRight == lastCountRight)
    { //wheel has not pulsed yet
        if(millis() > movementBuffer)
        { //if the robot not moved for duration
          movementBuffer = millis() + movementStuckBufferDelay;
          leds.fill(WHITE, 0, 4);
          leds.show();
          moveBackwards();
          wait(150);
          moveBackwardsRotate();
          wait(250);
        }
      }
      else
      {
        movementBuffer = millis() + movementStuckBufferDelay;
        lastCountLeft = countLeft;
        lastCountRight = countRight;
      }
    getDistanceFront();
    getDistanceLeft();
  }
  stopRobot();
}
void CountA()
{
  noInterrupts();
  countLeft++;
  interrupts();
}

void CountB()
{
  noInterrupts();
  countRight++;
  interrupts();
}

void getDistanceLeft()
{
  digitalWrite(trigPinLeft, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinLeft, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  durationLeft = pulseIn(echoPinLeft, HIGH);
  // Calculating the distance
  distanceLeft = durationLeft * 0.034 / 2;
}

void getDistanceFront()
{  
  digitalWrite(trigPinFront, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinFront, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  durationFront = pulseIn(echoPinFront, HIGH);
  // Calculating the distance
  distanceFront = durationFront * 0.034 / 2;
 
}
