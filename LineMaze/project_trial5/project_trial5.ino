//Libraries

#include <Adafruit_NeoPixel.h>
#include <QTRSensors.h> 

//Pins
#define PIN        7
#define NUMPIXELS  4

#define trigPin 2
#define echoPin 3 

const int numSensors = 8;
const int sensorPins[numSensors] = {A0, A1, A2, A3, A4, A5, A6, A7};

const int motorAPin1=11;
const int motorAPin2=10;
const int motorBPin1=6;
const int motorBPin2=5;

const int R1 = 9;
const int R2 = 8;

const int gripperPin = 12;

//Variables

volatile int countL = 0;
volatile int countR = 0;

volatile long duration;
volatile int distance;

bool wait = true;
bool set = false; //set is a boolean variable to check if the robot has done reading the sensorValue and calculate the rotator. sensors or not
bool solvedMaze = false; //solvedMaze is a boolean value to check whether the robot has come to the end of the maze or not yet.
bool endMaze = false; //endMaze is a variable to check if the robot has successfully solved the maze or not.

QTRSensors qtr;
const uint8_t SensorCount = 8;

uint16_t sensorValues[SensorCount];

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


void setup() {
  Serial.begin(9600);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7, A6, A5, A4, A3, A2, A1, A0}, SensorCount);

  pixels.begin();
  pixels.clear();
  pixels.show();

  pinMode(gripperPin, OUTPUT);
  
  for(int i = 0; i < numSensors; i++ ){
    pinMode(sensorPins[i], INPUT);
    }
  pinMode(motorAPin1, OUTPUT);
  pinMode(motorAPin2, OUTPUT);
  pinMode(motorBPin1, OUTPUT);
  pinMode(motorBPin2, OUTPUT);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(R1, INPUT_PULLUP);
  pinMode(R2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(9), rotationL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(8), rotationR, CHANGE);
  }

void loop() {
  qtr.read(sensorValues);

  if(wait)
  {
    detectObject();
  }
  else if (!set) 
  {
    start();
  }
  else if (!solvedMaze)
  {
    maze();
  }
  else if (!endMaze)
  {
    success();
  }
}

//Functions

void rotationL()
{
  countL++;
}

void rotationR()
{
  countR++;
}

//Functions to handle the logic to solve the line maze and pick up the flag
//The robot will go through 4 lines at the start space before starting solving the maze
void start()
{
  delay(3000);
  int lines = 0;
  forwardSlow();
  while(true)
  {
    qtr.read(sensorValues);
    forwardSlow();
    if(sensorValues[0] > 700 && sensorValues[1] > 700 && sensorValues[2] > 700 && sensorValues[3] > 700 && sensorValues[4] > 700 && sensorValues[5] > 700 && sensorValues[6] > 700 && sensorValues[7] > 700)
    {
      lines +=1;
      delay(200);
    }
    if(sensorValues[0] < 700 && sensorValues[1] < 700 && sensorValues[2] < 700 && sensorValues[3] > 700 && sensorValues[4] > 700 && sensorValues[5] < 700 && sensorValues[6] < 700 && sensorValues[7] < 700)
    {
      forward();
    }
    else if(sensorValues[0] < 700 && sensorValues[1] < 700 && sensorValues[2] < 700 && sensorValues[5] > 700 || sensorValues[6] > 700 || sensorValues[7] > 700)
    {
      adjustRight();
    }
    else if(sensorValues[0] > 700 || sensorValues[1] > 700 || sensorValues[2] > 700 && sensorValues[5] < 700 && sensorValues[6] < 700 && sensorValues[7] < 700)
    {
      adjustLeft();
    }
    if(lines > 4)
    {
      stop();
      delay(200);
      analogWrite(gripperPin, 40);
      delay(50);
      analogWrite(gripperPin, 0);
      delay(1000);
      turnLeft(33);
      break;
    }
  }
  set = true;
}

//Function to solve the maze
void maze()
{
  if(sensorValues[4] > 700 && sensorValues[5] > 700 && sensorValues[6] > 700 && sensorValues[7] > 700)
  {
    forward();
    delay(100);
    stop();

    while(true)
    {
      qtr.read(sensorValues);
      if(sensorValues[0] > 700 && sensorValues[1] > 700 && sensorValues[2] > 700 && sensorValues[3] > 700 && sensorValues[4] > 700 && sensorValues[5] > 700 && sensorValues[6] > 700 && sensorValues[7] > 700)
      {
        stop();
        delay(250);
        analogWrite(gripperPin, 200);
        solvedMaze = true;
        break;
      }
      else
      {
        turnRight(42);
        break;
      }
    }
  }
  else if(sensorValues[3] > 700 && sensorValues[4] > 700)
  {
    forward();
  }
  else if(sensorValues[5] > 700 || sensorValues[6] > 700 || sensorValues[7] > 700)
  {
    adjustRight();
  }
  else if(sensorValues[0] > 700 || sensorValues[1] > 700 || sensorValues[2] > 700)
  {
    adjustLeft();
  }
  else if(sensorValues[0] < 700 && sensorValues[1] < 700 && sensorValues[2] < 700 && sensorValues[3] < 700 && sensorValues[4] < 700 && sensorValues[5] < 700 && sensorValues[6] < 700 && sensorValues[7] < 700)
  {
    forward();
    delay(100);
    turnLeft2(26);

    while(true)
    {
      qtr.read(sensorValues);

      if(sensorValues[0] > 700 || sensorValues[1] > 700 || sensorValues[2] > 700 || sensorValues[3] > 700 || sensorValues[4] > 700 || sensorValues[5] > 700 || sensorValues[6] > 700 || sensorValues[7] > 700)
      {
        break;
      }
      if(sensorValues[0] < 700 && sensorValues[1] < 700 && sensorValues[2] < 700 && sensorValues[3] < 700 && sensorValues[4] < 700 && sensorValues[5] < 700 && sensorValues[6] < 700 && sensorValues[7] < 700)
      {
        spinLeft();
      }
    }
  }
}

void success()
{
  backward();
  delay(200);
  turnLeft2(135);
  stop();
  endMaze = true;
}
void forward()
{
    analogWrite(motorAPin1, 250);
    analogWrite(motorAPin2, 250);
    analogWrite(motorBPin1, 0);
    analogWrite(motorBPin2, 0);
}
void forwardSlow()
{
    analogWrite(motorAPin1, 140);
    analogWrite(motorAPin2, 130);
    analogWrite(motorBPin1, 0);
    analogWrite(motorBPin2, 0);
}
void backward()
{
    analogWrite(motorAPin1, 0);
    analogWrite(motorAPin2, 0);
    analogWrite(motorBPin1, 250);
    analogWrite(motorBPin2, 250);
}
void adjustLeft()
{
    analogWrite(motorAPin1, 130);
    analogWrite(motorAPin2, 250);
    analogWrite(motorBPin1, 0);
    analogWrite(motorBPin2, 0);
}
void adjustRight()
{
    analogWrite(motorAPin1, 250);
    analogWrite(motorAPin2, 130);
    analogWrite(motorBPin1, 0);
    analogWrite(motorBPin2, 0);
}
void stop()
{
    analogWrite(motorAPin1, 0);
    analogWrite(motorAPin2, 0);
    analogWrite(motorBPin1, 0);
    analogWrite(motorBPin2, 0);
}

void turnRight(int c)
{
  bool rotate = true;
  int cycles = c;
  countL = 0;
    
    while (rotate == true)
    {
      if(countL < cycles)
      {
        analogWrite(motorAPin1, 200);
        analogWrite(motorAPin2, 0);
        analogWrite(motorBPin1, 0);
        analogWrite(motorBPin2, 0);
      }
      else if(countL > cycles)
      {
        rotate = false;
      }
    }
}
void turnLeft(int c)
{
  bool rotate = true;
  int cycles = c;

  countR = 0;
    
    while (rotate == true)
    {
      if(countR < cycles)
      {
        analogWrite(motorAPin1, 0);
        analogWrite(motorAPin2, 200);
        analogWrite(motorBPin1, 0);
        analogWrite(motorBPin2, 0);
      }
      else if(countR > cycles)
      {
        rotate = false;
      }
    }
}

//Use both motors to turn left
void turnLeft2(int c)
{
  bool rotate = true;
  int cycles = c;

  countR = 0;
    
    while (rotate == true)
    {
      if(countR < cycles)
      {
        analogWrite(motorAPin1, 0);
        analogWrite(motorAPin2, 150);
        analogWrite(motorBPin1, 150);
        analogWrite(motorBPin2, 0);
      }
      else if(countR > cycles)
      {
        rotate = false;
      }
    }
}
void spinLeft()
{
        analogWrite(motorAPin1, 0);
        analogWrite(motorAPin2, 130);
        analogWrite(motorBPin1, 130);
        analogWrite(motorBPin2, 0);
}

void NeoPixel(int mode)
{
  switch(mode)
  {
    case 1:
      for(int i=0; i<NUMPIXELS; i++)
      {
        delay(5);

        pixels.clear();
        pixels.show();

        delay(2000);

        for(int pin=0; pin<NUMPIXELS; pin++)
        {
          pixels.setPixelColor(pin, pixels.Color(0, 150, 0));
        }

        pixels.show();
      }
      break;

    case 2:
      for(int i=0; i<NUMPIXELS; i++)
      {
        delay(5);
        pixels.clear();
        pixels.show();

        delay(500);
        pixels.setPixelColor(i, pixels.Color(0, 150, 0));
        pixels.setPixelColor(i+=1, pixels.Color(0, 150, 0));
        pixels.show();
      }
      break;

    default:
      break;
  }
}
int getDistance() 
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  return distance;
}

void detectObject()
{
  int distance = getDistance();

  if(distance < 25)
  {
    wait = false;
  } 
}
