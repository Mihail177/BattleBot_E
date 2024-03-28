#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>

boolean hasInitiatedStart = false;
boolean hasStarted = false;

const int lineSensors[] = {A0, A1, A2, A3, A4,A6, A7};
int lineSensorSensitivity = 700;

#define PIN 13
#define NUMPIXELS 4

#define LEFT_BACK_LED 0
#define LEFT_FRONT_LED 3
#define RIGHT_BACK_LED 1
#define RIGHT_FRONT_LED 2

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

//===[ Gripper ]===================================
const int GRIPPER_PIN=11;
const int GRIPPER_OPEN_PULSE=1600;
const int GRIPPER_CLOSE_PULSE=971;
const int GRIPPER_PULSE_REPEAT=10;
//===[ Time variables ]=============================
unsigned long time;


//===[ DistanceSenzors ]============================

long durationFront;
int distanceFront;
const int trigPinFront = 8;
const int echoPinFront = 12;

long durationLeft;
int distanceLeft;
const int trigPinLeft = 4;
const int echoPinLeft = A5;
int distanceOverride = 0;

//===[ Motor pins ]======================

const int motorRightBackwards=10;
const int motorRightForward=9;  
const int motorLeftBackwards=5;
const int motorLeftForward=6;
const int movementStuckBufferDelay=1000;

//===[ Motor tests for pulses ]====================

const int motor_R1=2;
const int motor_R2=3;
int countLeft=0;
int countRight=0;
int countsLeft=0,previousCountLeft;
int countsRight=0,previousCountRight;

//===[ Functions ]=================================

boolean isBlackZone(){
  int count = 0;
  for(int i = 0; i < 7; i++){
    if(analogRead(lineSensors[i]) > lineSensorSensitivity){
      count ++;
    }
  }
  if(count < 4){
    return false;
  }
  return true;
}

void setup_motor_pins()
{ 
  pinMode(motorRightBackwards,OUTPUT);
  pinMode(motorRightForward,OUTPUT);
  pinMode(motorLeftBackwards,OUTPUT);
  pinMode(motorLeftForward,OUTPUT);
}
void moveForward() // shows green
{
  strip.setPixelColor(LEFT_BACK_LED, strip.Color(255, 0, 0));
  strip.setPixelColor(LEFT_FRONT_LED, strip.Color(255, 0, 0));
  strip.setPixelColor(RIGHT_BACK_LED, strip.Color(255, 0, 0));
  strip.setPixelColor(RIGHT_FRONT_LED, strip.Color(255, 0, 0));
  strip.show();
  
  analogWrite(motorLeftForward,230);
  analogWrite(motorRightForward,255);
  analogWrite(motorRightBackwards,0);
  analogWrite(motorLeftBackwards,0);
}

void moveBackwardsRotateRight()
{
  strip.setPixelColor(LEFT_BACK_LED, strip.Color(204, 204, 0));
  strip.setPixelColor(LEFT_FRONT_LED, strip.Color(204, 204, 0));
  strip.setPixelColor(RIGHT_BACK_LED, strip.Color(0, 0, 255));
  strip.setPixelColor(RIGHT_FRONT_LED, strip.Color(0, 0, 255));
  strip.show();
  
  analogWrite(motorRightBackwards,255);
  analogWrite(motorLeftBackwards,90);
  analogWrite(motorRightForward, 0);
  analogWrite(motorLeftForward,0);
}

void moveBackwardsRotateLeft()
{
  strip.setPixelColor(LEFT_BACK_LED, strip.Color(0, 0, 255));
  strip.setPixelColor(LEFT_FRONT_LED, strip.Color(0, 0, 255));
  strip.setPixelColor(RIGHT_BACK_LED, strip.Color(204, 204, 0));
  strip.setPixelColor(RIGHT_FRONT_LED, strip.Color(204, 204, 0));
  strip.show();
  analogWrite(motorRightBackwards,90);
  analogWrite(motorLeftBackwards,255);
  analogWrite(motorRightForward, 0);
  analogWrite(motorLeftForward,0);
}

void moveBackwards()
{
  strip.setPixelColor(LEFT_BACK_LED, strip.Color(0, 0, 255));
  strip.setPixelColor(LEFT_FRONT_LED, strip.Color(0, 0, 255));
  strip.setPixelColor(RIGHT_BACK_LED, strip.Color(0, 0, 255));
  strip.setPixelColor(RIGHT_FRONT_LED, strip.Color(0, 0, 255));
  strip.show();
  
  analogWrite(motorRightBackwards,240);
  analogWrite(motorLeftBackwards, 255);
  analogWrite(motorRightForward, 0);
  analogWrite(motorLeftForward,0);
}

void stopRobot() {
  strip.setPixelColor(LEFT_BACK_LED, strip.Color(0, 255, 0));
  strip.setPixelColor(LEFT_FRONT_LED, strip.Color(0, 255, 0));
  strip.setPixelColor(RIGHT_BACK_LED, strip.Color(0, 255, 0));
  strip.setPixelColor(RIGHT_FRONT_LED, strip.Color(0, 255, 0));
  strip.show();
  
  analogWrite(motorRightBackwards,0);
  analogWrite(motorRightForward, 0);
  analogWrite(motorLeftBackwards, 0);
  analogWrite(motorLeftForward,0);
}
void turnLeft()
{
  strip.setPixelColor(LEFT_BACK_LED, strip.Color(0, 255, 0));
  strip.setPixelColor(LEFT_FRONT_LED, strip.Color(0, 255, 0));
  strip.setPixelColor(RIGHT_BACK_LED, strip.Color(255, 0, 0));
  strip.setPixelColor(RIGHT_FRONT_LED, strip.Color(255, 0, 0));
  strip.show();
  
  analogWrite(motorLeftBackwards,0);
  analogWrite(motorLeftForward, 0);
  analogWrite(motorRightForward, 255);
  analogWrite(motorRightBackwards,0);
}
void rotateOnAxis()
{
  strip.setPixelColor(LEFT_BACK_LED, strip.Color(0, 0, 255));
  strip.setPixelColor(LEFT_FRONT_LED, strip.Color(0, 0, 255));
  strip.setPixelColor(RIGHT_BACK_LED, strip.Color(255, 0, 0));
  strip.setPixelColor(RIGHT_FRONT_LED, strip.Color(255, 0, 0));
  strip.show();
  
  analogWrite(motorLeftBackwards,240);
  analogWrite(motorLeftForward, 0);
  analogWrite(motorRightForward, 255);
  analogWrite(motorRightBackwards,0);
}
void rotateCounterAxis()
{
  strip.setPixelColor(LEFT_BACK_LED, strip.Color(255, 0, 0));
  strip.setPixelColor(LEFT_FRONT_LED, strip.Color(255, 0, 0));
  strip.setPixelColor(RIGHT_BACK_LED, strip.Color(0, 0, 255));
  strip.setPixelColor(RIGHT_FRONT_LED, strip.Color(0, 0, 255));
  strip.show();
  
  analogWrite(motorLeftBackwards,0);
  analogWrite(motorLeftForward, 240);
  analogWrite(motorRightForward, 0);
  analogWrite(motorRightBackwards,255);
}
void rotatePulses(int nrOfPulses)
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
  wait(250);
  while ((countLeft<nrOfPulses && countRight<nrOfPulses) && distanceLeft>=15 && isActive)
    {
      if(countLeft == lastCountLeft && countRight == lastCountRight)
      { //wheel has not pulsed yet
        if(millis() > movementBuffer && distanceLeft < 3)
        { //if not moved for duration
          movementBuffer = millis() + movementStuckBufferDelay;
          moveBackwardsRotateRight();
          wait(400);
        }
        else if (millis() > movementBuffer && distanceLeft >= 3)
        {
          movementBuffer = millis() + movementStuckBufferDelay;
          moveBackwardsRotateLeft();
          wait(400);
        }
      }
      else{
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
  moveForward();
  while ((countLeft<nrOfPulses && countRight<nrOfPulses) && isActive)
  {
    if(countLeft == lastCountLeft || countRight == lastCountRight){ //wheel has not pulsed yet and was && before
        if(millis() > movementBuffer){ //if not moved for duration
          movementBuffer = millis() + movementStuckBufferDelay;
          moveBackwards();
          wait(500);
        }
      }
      else{
        movementBuffer = millis() + movementStuckBufferDelay;
        lastCountLeft = countLeft;
        lastCountRight = countRight;
      }
    getDistanceFront();
  }
  stopRobot();
}
void showNrOfPulse()
{
 Serial.print(countLeft);
 Serial.print(" ");
 Serial.print(countRight);
 Serial.println();
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

void wait(int waitingTime) {
  time = millis();
  while(millis() < time + waitingTime){
    }
}

void gripperServo(int pulse)
{
    for(int i = 0; i < GRIPPER_PULSE_REPEAT;i++)
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
//===[SETUP ]============================

void setup() {
  strip.begin();
  strip.show();
  
  pinMode(motor_R1, INPUT_PULLUP);
  pinMode(motor_R2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(motor_R1),CountA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor_R2),CountB,CHANGE);
  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
  pinMode(trigPinLeft, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinLeft, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600);
  countLeft=0;
  countRight=0; 
}

//===[ LOOP ]============================

void loop()
{   // first - green , second - red, third - blue
  
  if(hasStarted){
   getDistanceLeft();
   getDistanceFront();
   if (distanceLeft<=15&&distanceFront>=15)
   {
    moveForwardOnPulses(31);
    stopRobot();
    wait(200);
   }
   else if(distanceLeft>15 && distanceFront>15)
    {
      rotatePulses(31);
      stopRobot();
      wait(200);
      moveForwardOnPulses(25);
//      rotatePulses(50);
    }
   else if(distanceLeft<20&&distanceFront<=20)
   {
    getDistanceFront();
    rotateCounterAxis();
    wait(400); 
    moveForwardOnPulses(31);
    }
    else{
      moveBackwards();
      wait(350);
    }
  }
  else {
    if(hasInitiatedStart == true){
      moveForward();
      wait(700);
      closeGripper();
      countLeft=0;
      countRight=0;
      rotatePulses(70);
      moveForwardOnPulses(50);
      hasStarted = true;
    }
    else {
      wait(1000);
      getDistanceFront();
      if(distanceFront < 15){
        openGripper();
        wait(1500);
        hasInitiatedStart = true;
      }
    }
  }
}
